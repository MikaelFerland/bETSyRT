/*
    Title:    bETSyRT.c
    Authors:  Mikael Ferland, Joël Brisson
    Date:     11/2015
    Purpose:  Control a robot through UART and SPI
    needed
    Software: AVR-GCC to compile
    needed
    Hardware: ATMega32 on STK500 board
    Note:     
*/


#include "includes.h"

/* Constants ------------------------------------------------ */
#define PI      (3.1415926535897932384626433832795)
#define START_DEBUG 	0xFE
#define STOP_DEBUG		0xFF
#define SET_DIR_L1		PIND2
#define SET_DIR_L2		PIND3
#define SET_DIR_R1		PIND6
#define SET_DIR_R2		PIND7
#define GET_L_DIR_PIN	PINA2
#define GET_R_DIR_PIN	PINA3
#define SPEED_REMOTE_MAX 0xC8
#define CALIB_PIN		PINA4

/* LED-------------------------------------------------------- */
#define Command_LED 	1
#define RObstacle_LED 	2
#define RSonar_LED 		3
#define LObstacle_LED	4
#define LSonar_LED 		5
#define WAIT_LED 		6
#define RUN_LED 		7


/* Macro------------------------------------------------------ */
#define PRINT_DEBUG(string) print((PGM_P) string)

/* Stack Counter Define -------------------------*/
#define OS_STANDARD_STK_SIZE 100
#define OS_SMALL_STK_SIZE 50

OS_STK TIME_TO_COMPUTE_PWM_STK	[OS_STANDARD_STK_SIZE];
OS_STK PACKET_READY_STK			[OS_STANDARD_STK_SIZE];
OS_STK TIME_TO_PING_STK			[OS_SMALL_STK_SIZE];
OS_STK UPDATE_LED_STK			[OS_SMALL_STK_SIZE];
OS_STK SEND_DEBUG_STK			[OS_STANDARD_STK_SIZE];

/* OS EVENT ----------------------------------*/
OS_EVENT *TIME_TO_COMPUTE_PWM_SEM;
OS_EVENT *PACKET_READY_SEM;
OS_EVENT *TIME_TO_PING_SEM;
OS_EVENT *UPDATE_LED_SEM;
OS_EVENT *SEND_DEBUG_SEM;


/*Global Variables ------------------------------------------- */
volatile u08 TIME_TO_COMPUTE_PWM;
volatile u08 CURRENT_CHANNEL;
volatile u08 L_DIR_SENSOR, R_DIR_SENSOR;
volatile u08 STAB;
volatile u08 LED_Status = 0x00, OLD_LED_Status = 0x00, LED_Counter = 0x00;

volatile s08 R_SIGN, L_SIGN;

volatile u16 V_L_MOTOR_SENSOR_RAW, V_R_MOTOR_SENSOR_RAW;

volatile int V_L_MOTOR_SENSOR_ACCUMULATOR, V_R_MOTOR_SENSOR_ACCUMULATOR;

volatile float V_L_MOTOR_SENSOR_MEAN, V_R_MOTOR_SENSOR_MEAN;
volatile u16 L_COUNTER_FOR_MEAN, R_COUNTER_FOR_MEAN;

volatile u08 Ping_Side = 0;
volatile u08 Ping_Sensor_MSB, Ping_Sensor_LSB;
volatile u16 R_Ping_Sense, L_Ping_Sense, Closest_Sense;
volatile u08 Time_To_Ping = 0;

float   SPEED_REMOTE, ANGLE_REMOTE;
float	SPEED_SETPOINT, ANGLE_SETPOINT;
float	V_L_MOTOR_SENSOR_ENG, V_R_MOTOR_SENSOR_ENG;
float 	DUTY_R, DUTY_L;

float	L_slope_P, L_slope_N, R_slope_P, R_slope_N;

int		L_VMIN_N,L_VMAX_N, L_VMIN_P, L_VMAX_P;
int		R_VMIN_N,R_VMAX_N, R_VMIN_P, R_VMAX_P;

u16   DUTY_L_REG, DUTY_R_REG;

u16	  TT;

/* TASK ------------------------------------------------------ */

static void PACKET_READY_TASK(void *p_arg);
static void TIME_TO_PING_TASK(void *p_arg);
static void TIME_TO_COMPUTE_PWM_TASK(void *p_arg);
static void UPDATE_LED_TASK(void *p_arg);
static void SEND_DEBUG_TASK(void *p_arg);

/* Functions ------------------------------------------------- */
extern void InitOSTimer(void);

//Print information on the remote consol
void print(PGM_P string){
		UART_DisableEcho();
		UART_SendByte(START_DEBUG);
		PRINT(string);	
		UART_SendByte(STOP_DEBUG);
		UART_EnableEcho();
}

//Set left motor to neutral
void l_motorToNeural(){
	PORTD &= ~( (1 << SET_DIR_L1) | (1 << SET_DIR_L2) );
}

//Set right motor to neutral
void r_motorToNeural(){
	PORTD &= ~( (1 << SET_DIR_R1) | (1 << SET_DIR_R2) );
}

//Set all motor to neutral
void robotToNeutral(){
	PORTD &= ~( (1 << SET_DIR_L1) | (1 << SET_DIR_L2) | (1 << SET_DIR_R1) | (1 << SET_DIR_R2) );
}

//Set left motor to forward
void l_forward_motor(){
	PORTD |=  (1 << SET_DIR_L1);
	PORTD &= ~(1 << SET_DIR_L2);
}

//Set right motor to forward
void r_forward_motor(){
	PORTD |=  (1 << SET_DIR_R1);
	PORTD &= ~(1 << SET_DIR_R2);
}

//Set left motor to reverse
void l_reverse_motor(){
	PORTD &= ~(1 << SET_DIR_L1);
	PORTD |=  (1 << SET_DIR_L2);
}

//Set right motor to reverse
void r_reverse_motor(){
	PORTD &= ~(1 << SET_DIR_R1);
	PORTD |=  (1 << SET_DIR_R2);
}

//Set all motor to stop
void stopRobot(){
	OCR1A = 0;
	OCR1B = 0;
	PORTD |= ( (1 << SET_DIR_L1) | (1 << SET_DIR_L2) | (1 << SET_DIR_R1) | (1 << SET_DIR_R2) );
}

//Simple short delay of 1.5mS
void delay(){
	u16 count = 1000;
	u16 count1 = 25;
	while(count != 0){
		count--;

		while(count1 != 0){
			count1--;
		}
		count1 = 25;
	}
}

//Wait for 11 ADC Sample on both motor side
void waitForSample(){
	while((L_COUNTER_FOR_MEAN < 12) && (R_COUNTER_FOR_MEAN < 12));
}

//Reset all value use for Mean
void resetMeanValue(){
	V_L_MOTOR_SENSOR_ACCUMULATOR =0;
	V_R_MOTOR_SENSOR_ACCUMULATOR =0;
	V_L_MOTOR_SENSOR_MEAN = 0.0;
	V_R_MOTOR_SENSOR_MEAN = 0.0;
	L_COUNTER_FOR_MEAN =0;
	R_COUNTER_FOR_MEAN =0;
}

//Calibration of max, neutral and min value for the right and left motor
void calibrate(){

	//Vmax+ motors
	stopRobot();
	delay();
	
	PORTA |=  (1 << CALIB_PIN);
	l_forward_motor();
	r_forward_motor();
	delay();

	resetMeanValue();
	waitForSample();

	//Calculate mean value
	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));
	
	L_VMAX_P = V_L_MOTOR_SENSOR_MEAN;
	R_VMAX_P = V_R_MOTOR_SENSOR_MEAN;
	PORTA &= ~(1 << CALIB_PIN);

	//Vzero+ motors
	stopRobot();
	delay();

	resetMeanValue();
	waitForSample();

	//Calculate mean value
	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));

	L_VMIN_P = V_L_MOTOR_SENSOR_MEAN;
	R_VMIN_P = V_R_MOTOR_SENSOR_MEAN;
	
	//Vmax- motors
	stopRobot();
	delay();

	PORTA |=  (1 << CALIB_PIN);
	l_reverse_motor();
	r_reverse_motor();
	delay();

	resetMeanValue();
	waitForSample();

	//Calculate mean value
	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));

	L_VMAX_N = V_L_MOTOR_SENSOR_MEAN;
	R_VMAX_N = V_R_MOTOR_SENSOR_MEAN;
	PORTA &= ~(1 << CALIB_PIN);

	//Vzero- motors
	stopRobot();
	delay();
	
	resetMeanValue();
	waitForSample();

	//Calculate mean value
	V_L_MOTOR_SENSOR_MEAN = (float)(V_L_MOTOR_SENSOR_ACCUMULATOR / ((int)L_COUNTER_FOR_MEAN));
	V_R_MOTOR_SENSOR_MEAN = (float)(V_R_MOTOR_SENSOR_ACCUMULATOR / ((int)R_COUNTER_FOR_MEAN));
	
	L_VMIN_N = V_L_MOTOR_SENSOR_MEAN;
	R_VMIN_N = V_R_MOTOR_SENSOR_MEAN;

	L_slope_P = (1.0 / (L_VMAX_P - L_VMIN_P));
	R_slope_P = (1.0 / (R_VMAX_P - R_VMIN_P));
	L_slope_N = (1.0 / (L_VMAX_N - L_VMIN_N));
	R_slope_N = (1.0 / (R_VMAX_N - R_VMIN_N));

	resetMeanValue();
}

//Update the LED state
void UpdateLED()
{
	// when a flashing led has his status bit set to '1',
	// toggle the LED output then put his status to '0'
	if((LED_Status & 0x02))
	{
		//toggle PIN State
		PORTB = (PORTB ^ 0x02);
	}

	if((LED_Status & 0x08))
	{
		//toggle PIN State
		PORTB = (PORTB ^ 0x08);
	}

	if((LED_Status & 0x20))
	{
		//toggle PIN State
		PORTB = (PORTB ^ 0x20);
	}

	// Mask all LED that does not flash
	// then write them to the LED PORT
	PORTB = (0xD4 | PORTB) & ~(0xD4 & LED_Status);

	LED_Status = LED_Status & 0xD4;
	OLD_LED_Status = LED_Status;
}

//Speed command must be between -1 and 1
//Return 33 to 68 for speed_command > 0
u08 SetSonarRange(float speed_command){	
	return  (0 <= speed_command) ? (u08)(( (68.7274-33.8837) * speed_command) + 33.8837) : 0;
}

//Convert speed command into a gain in range of 8 to 12
//Speed command must be between -1 and 1
//Return 8 to 12 for speed_command > 0
u08 SetSonarGain(float speed_command){
	return  (0 <= speed_command) ? (u08)((4.0 * speed_command) + 8.0) : 0;
}

//Interrupt for: Timer 1 flag
SIGNAL(SIG_OVERFLOW1)
{
	OS_INT_ENTER();

	TIME_TO_COMPUTE_PWM = 1;
	Time_To_Ping++;

	OS_INT_EXIT();
}

//Interrupt for: End of ADC conversion
SIGNAL(SIG_ADC)
{
	OS_INT_ENTER();

	if (STAB == 0){
		if(CURRENT_CHANNEL == 0){
			L_DIR_SENSOR = PINA & (1 << GET_L_DIR_PIN);
			L_SIGN = (L_DIR_SENSOR ==0) ? 1 : -1;

			V_L_MOTOR_SENSOR_RAW =  ADC & (0x3FF); //10 bits mask

			if (L_SIGN > 0){
				V_L_MOTOR_SENSOR_ACCUMULATOR += ((int)V_L_MOTOR_SENSOR_RAW);
			}
			else{
				V_L_MOTOR_SENSOR_ACCUMULATOR -= ((int)V_L_MOTOR_SENSOR_RAW);
			}
			
			L_COUNTER_FOR_MEAN++;
		}
		else{
			R_DIR_SENSOR = PINA & (1 << GET_R_DIR_PIN);			
			R_SIGN = (R_DIR_SENSOR ==0) ? 1 : -1;

			V_R_MOTOR_SENSOR_RAW =  ADC & (0x3FF); //10 bits mask

			if(R_SIGN >0){
				V_R_MOTOR_SENSOR_ACCUMULATOR += ((int)V_R_MOTOR_SENSOR_RAW);
			}
			else{
				V_R_MOTOR_SENSOR_ACCUMULATOR -= ((int)V_R_MOTOR_SENSOR_RAW);
			}
			
			R_COUNTER_FOR_MEAN++;

		}
		
		CURRENT_CHANNEL = !(CURRENT_CHANNEL & 0x01);
		ADMUX = CURRENT_CHANNEL;
				
		STAB = 3;
		
	}
	else{
		STAB--;
	}

	OS_INT_EXIT();
}


void main ()
{
	OSInit();
	/*OS Task Create--------------------------------*/
	OSTaskCreate(PACKET_READY_TASK, NULL, (OS_STK *) &PACKET_READY_STK[OS_STANDARD_STK_SIZE-1], 4);
	OSTaskCreate(TIME_TO_PING_TASK, NULL, (OS_STK *) &TIME_TO_PING_STK[OS_SMALL_STK_SIZE-1], 5);
	OSTaskCreate(TIME_TO_COMPUTE_PWM_TASK, NULL, (OS_STK *) &TIME_TO_COMPUTE_PWM_STK[OS_STANDARD_STK_SIZE-1], 6);
	OSTaskCreate(UPDATE_LED_TASK, NULL, (OS_STK *) &UPDATE_LED_STK[OS_SMALL_STK_SIZE-1], 7);
	OSTaskCreate(SEND_DEBUG_TASK, NULL, (OS_STK *) &SEND_DEBUG_STK[OS_SMALL_STK_SIZE-1], 8);
	
	
	/*OS Sem Create-----------------------------*/
	PACKET_READY_SEM = OSSemCreate(0);
	TIME_TO_PING_SEM = OSSemCreate(0);
	TIME_TO_COMPUTE_PWM_SEM = OSSemCreate(0);
	UPDATE_LED_SEM = OSSemCreate(0);
	SEND_DEBUG_SEM = OSSemCreate(0);


	/* Initialise Hardware */

	//Calibration output at PINA4
	DDRA = 0x10;

	//**DDRD: 1 = OUTPUT**
    //Pin 7 to 2 on Port D are output
	DDRD = 0xFC;

	//Pin 7 to 1 on Port B are Output. *LED*
	DDRB = 0xFE; //NC on port 0 pin 0, keep it input
	
	PORTB=0xFF;

	resetMeanValue();

	CURRENT_CHANNEL = 0;
	DUTY_R = 0.0;
	DUTY_L = 0.0;
	STAB = 30;
	TT=500;

	/* Initialisation */
	ADC_Init();
	timer_Init();

	//Enable Interrupt		
	sei();
	
	calibrate();

	TWIInit();
	UART_Init();

	l_forward_motor();
	r_forward_motor();

	OSStart();
}

static void PACKET_READY_TASK(void *p_arg)
{
 	INT8U   err;

    (void)p_arg;

	while(TRUE){
	
	}
}

static void TIME_TO_PING_TASK(void *p_arg)
{
 	INT8U   err;

    (void)p_arg;

	while(TRUE){
	
	}
}

static void TIME_TO_COMPUTE_PWM_TASK(void *p_arg)
{
 	INT8U   err;

    (void)p_arg;

	while(TRUE){
	
	}
}

static void UPDATE_LED_TASK(void *p_arg)
{
 	INT8U   err;

    (void)p_arg;
	
	while(TRUE){
	
	}
}

static void SEND_DEBUG_TASK(void *p_arg)
{
 	INT8U   err;

    (void)p_arg;

	while(TRUE){
	
	}
}

