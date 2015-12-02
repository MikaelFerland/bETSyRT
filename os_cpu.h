/*
*********************************************************************************************************
*                                               uC/OS-II
*                                        The Real-Time Kernel
*
*                                         ATMega32 Specific code
*
* File         : OS_CPU.H
* By           : Bruno De Kelper
* Port Version : V1.0
*
*********************************************************************************************************
*/

/* Definition moved here so it can be used in the assembler file OS_CPU_A.ASM 							*/
/* See below for the meaning of this define 															*/

#define  OS_CRITICAL_METHOD    2


#ifdef  OS_CPU_GLOBALS
#define OS_CPU_EXT
#else
#define OS_CPU_EXT  extern
#endif

/*
**********************************************************************************************************
*                                              DATA TYPES
*                                         (Compiler Specific)
**********************************************************************************************************
*/

typedef unsigned char  BOOLEAN;
typedef unsigned char  INT8U;                       /* Unsigned  8 bit quantity                         */
typedef signed   char  INT8S;                       /* Signed    8 bit quantity                         */
typedef unsigned int   INT16U;                      /* Unsigned 16 bit quantity                         */
typedef signed   int   INT16S;                      /* Signed   16 bit quantity                         */
typedef unsigned long  INT32U;                      /* Unsigned 32 bit quantity                         */
typedef signed   long  INT32S;                      /* Signed   32 bit quantity                         */
typedef float          FP32;                        /* Single precision floating point                  */

typedef unsigned char  OS_STK;                      /* Each stack entry is 8-bit wide                   */

/*
**********************************************************************************************************
*                                       Atmel AVR
*
*
* Method #1:  Disable/Enable interrupts using simple instructions.  After critical section, interrupts
*             will be enabled even if they were disabled before entering the critical section.
*
* Method #2:  Disable/Enable interrupts by preserving the state of interrupts. In other words, if
*             interrupts were disabled before entering the critical section, they will be disabled when
*             leaving the critical section. Here's what happens in the macro below :
*
*             OS_ENTER_CRITICAL:
*
*               input SREG to a free register, Rn
*				push Rn
*
*             OS_EXIT_CRITICAL:
*
*				pop	to a free register, Rn
*				output Rn to SREG
*
*
**********************************************************************************************************
*/

#if      OS_CRITICAL_METHOD == 1
#define  OS_ENTER_CRITICAL()    cli();	/* Disable interrupts						*/
#define  OS_EXIT_CRITICAL()     sei();    /* Enable  interrupts						*/
#endif

#if      OS_CRITICAL_METHOD == 2
#define  OS_ENTER_CRITICAL()    {	\
									__asm__ __volatile__ ("PUSH  R16"::);  \
									__asm__ __volatile__ ("IN    R16,0x3F"::);  \
									__asm__ __volatile__ ("PUSH  R16"::);  \
									cli();  \
								}
#define  OS_EXIT_CRITICAL()     {	\
									__asm__ __volatile__ ("POP  R16"::);  \
									__asm__ __volatile__ ("OUT  0x3F,R16"::);  \
									__asm__ __volatile__ ("POP  R16"::);  \
								}
#endif

#if      OS_CRITICAL_METHOD == 3
#define  OS_ENTER_CRITICAL()	{	cpu_sr = SREG; \
									cli();	\	  /* Disable interrupts  */
								}
#define  OS_EXIT_CRITICAL()		{	SREG = cpu_sr;	} /* Enable  interrupts  */
#endif

/*
**********************************************************************************************************
*                           AVR Miscellaneous
**********************************************************************************************************
*/

#define  OS_STK_GROWTH      1                       /* Stack grows from HIGH to LOW memory on AVR       */

#define  OS_TASK_SW()       OSCtxSw()

#ifdef SIGNAL
#undef SIGNAL
#ifdef __cplusplus
#define SIGNAL(signame)					\
extern "C" void signame(void);				\
void signame (void) __attribute__ ((naked));    \
void signame (void) __attribute__ ((signal));		\
void signame (void)
#else
#define SIGNAL(signame)					\
void signame (void) __attribute__ ((naked));    \
void signame (void) __attribute__ ((signal));		\
void signame (void)
#endif

#define OS_INT_ENTER()	\
			 __asm__ __volatile__ ("PUSH  R18"::);  \
			 __asm__ __volatile__ ("PUSH  R19"::);  \
			 __asm__ __volatile__ ("IN    R18,0x3F"::);  \
			 __asm__ __volatile__ ("ORI   R18,0x80"::);  \
			 __asm__ __volatile__ ("PUSH  R18"::);  \
			 __asm__ __volatile__ ("PUSH  R0"::);  \
			 __asm__ __volatile__ ("PUSH  R1"::);  \
			 __asm__ __volatile__ ("PUSH  R20"::);  \
			 __asm__ __volatile__ ("PUSH  R21"::);  \
			 __asm__ __volatile__ ("PUSH  R22"::);  \
			 __asm__ __volatile__ ("PUSH  R23"::);  \
			 __asm__ __volatile__ ("PUSH  R24"::);  \
			 __asm__ __volatile__ ("PUSH  R25"::);  \
			 __asm__ __volatile__ ("PUSH  R26"::);  \
			 __asm__ __volatile__ ("PUSH  R27"::);  \
			 __asm__ __volatile__ ("PUSH  R30"::);  \
			 __asm__ __volatile__ ("PUSH  R31"::);  \
			if (OSIntNesting == 0) {	\
				__asm__ __volatile__ ("PUSH  R2"::);  \
				__asm__ __volatile__ ("PUSH  R3"::);  \
				__asm__ __volatile__ ("PUSH  R4"::);  \
				__asm__ __volatile__ ("PUSH  R5"::);  \
				__asm__ __volatile__ ("PUSH  R6"::);  \
				__asm__ __volatile__ ("PUSH  R7"::);  \
				__asm__ __volatile__ ("PUSH  R8"::);  \
				__asm__ __volatile__ ("PUSH  R9"::);  \
				__asm__ __volatile__ ("PUSH  R10"::);  \
				__asm__ __volatile__ ("PUSH  R11"::);  \
				__asm__ __volatile__ ("PUSH  R12"::);  \
				__asm__ __volatile__ ("PUSH  R13"::);  \
				__asm__ __volatile__ ("PUSH  R14"::);  \
				__asm__ __volatile__ ("PUSH  R15"::);  \
				__asm__ __volatile__ ("PUSH  R16"::);  \
				__asm__ __volatile__ ("PUSH  R17"::);  \
				__asm__ __volatile__ ("PUSH  R28"::);  \
				__asm__ __volatile__ ("PUSH  R29"::);  \
			}	\
			OSIntNesting++;

	 
	

#define OS_INT_EXIT()	\
			cli();	\
			OSIntExit();	\
			if (OSIntNesting == 0) {	\
				__asm__ __volatile__ ("POP  R29"::);  \
				__asm__ __volatile__ ("POP  R28"::);  \
				__asm__ __volatile__ ("POP  R17"::);  \
				__asm__ __volatile__ ("POP  R16"::);  \
				__asm__ __volatile__ ("POP  R15"::);  \
				__asm__ __volatile__ ("POP  R14"::);  \
				__asm__ __volatile__ ("POP  R13"::);  \
				__asm__ __volatile__ ("POP  R12"::);  \
				__asm__ __volatile__ ("POP  R11"::);  \
				__asm__ __volatile__ ("POP  R10"::);  \
				__asm__ __volatile__ ("POP  R9"::);  \
				__asm__ __volatile__ ("POP  R8"::);  \
				__asm__ __volatile__ ("POP  R7"::);  \
				__asm__ __volatile__ ("POP  R6"::);  \
				__asm__ __volatile__ ("POP  R5"::);  \
				__asm__ __volatile__ ("POP  R4"::);  \
				__asm__ __volatile__ ("POP  R3"::);  \
				__asm__ __volatile__ ("POP  R2"::);  \
			}	\
			__asm__ __volatile__ ("POP  R31"::);  \
			__asm__ __volatile__ ("POP  R30"::);  \
			__asm__ __volatile__ ("POP  R27"::);  \
			__asm__ __volatile__ ("POP  R26"::);  \
			__asm__ __volatile__ ("POP  R25"::);  \
			__asm__ __volatile__ ("POP  R24"::);  \
			__asm__ __volatile__ ("POP  R23"::);  \
			__asm__ __volatile__ ("POP  R22"::);  \
			__asm__ __volatile__ ("POP  R21"::);  \
			__asm__ __volatile__ ("POP  R20"::);  \
			__asm__ __volatile__ ("POP  R1"::);  \
			__asm__ __volatile__ ("POP  R0"::);  \
			__asm__ __volatile__ ("POP  R18"::);  \
			__asm__ __volatile__ ("ANDI R18,0x7F"::);  \
			__asm__ __volatile__ ("OUT  0x3F,R18"::);  \
			__asm__ __volatile__ ("POP  R19"::);  \
			__asm__ __volatile__ ("POP  R18"::);	\
			__asm__ __volatile__ ("RETI"::);

#endif















