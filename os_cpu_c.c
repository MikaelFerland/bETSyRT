/*
**********************************************************************************************************
*                                               uC/OS-II
*                                         The Real-Time Kernel
*
*                                           ATMega32 Specific code
*
* File         : OS_CPU_C.C
* By           : Bruno De Kelper
* Port Version : V1.0
*
**********************************************************************************************************
*/

#define  OS_CPU_GLOBALS
#include "includes.h"

#define PUSH_ALL() {  \
			 __asm__ __volatile__ ("PUSH  R18"::);  \
			 __asm__ __volatile__ ("PUSH  R19"::);  \
			 __asm__ __volatile__ ("IN    R18,0x3F"::);  \
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
		     }

#define POP_ALL() {  \
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
			__asm__ __volatile__ ("OUT  0x3F,R18"::);  \
			__asm__ __volatile__ ("POP  R19"::);  \
			__asm__ __volatile__ ("POP  R18"::);	\
		    }

/*
**********************************************************************************************************
*                                        INITIALIZE A TASK'S STACK
*
* Description: This function is called by either OSTaskCreate() or OSTaskCreateExt() to initialize the
*              stack frame of the task being created. This function is highly processor specific.
*
* Arguments  : task          is a pointer to the task code
*
*              pdata         is a pointer to a user supplied data area that will be passed to the task
*                            when the task first executes.
*
*              ptos          is a pointer to the top of stack. It is assumed that 'ptos' points to the
*                            highest valid address on the stack.
*
*              opt           specifies options that can be used to alter the behavior of OSTaskStkInit().
*                            (see uCOS_II.H for OS_TASK_OPT_???).
*
* Returns    : Always returns the location of the new top-of-stack' once the processor registers have
*              been placed on the stack in the proper order.
*
* Note(s)    : Interrupts are enabled when your task starts executing. You can change this by setting the
*              SREG to 0x00 instead. In this case, interrupts would be disabled upon task startup. The
*              application code would be responsible for enabling interrupts at the beginning of the task
*              code. You will need to modify OSTaskIdle() and OSTaskStat() so that they enable
*              interrupts. Failure to do this will make your system crash!
*
*              The AVR return stack is placed 64 bytes above the start of the (512 byte) stack. This
*              provides 32 levels of function call nesting which should be more than enough for most
*              applications (see the "read.me" file for more info).
*
**********************************************************************************************************
*/

OS_STK   *OSTaskStkInit(void (*task)(void *pd), void *pdata, OS_STK *ptos, INT16U opt)
{
    INT8U  *stk;
    INT16U  tmp;

    opt     = opt;                          /* 'opt' is not used, prevent warning                       */
    stk     = (INT8U *)ptos;				/* AVR return stack ("hardware stack")          			*/
    tmp     = (INT16U)task;

	/* "push" initial register values onto the stack */

    *stk-- = (INT8U)tmp;                   /* Put task start address on top of stack          			*/
    *stk-- = (INT8U)(tmp >> 8);

    *stk-- = (INT8U)0x00;                   /* R18  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R19  = 0x00                                               */
    *stk-- = (INT8U)0x80;                   /* SREG = Interrupts enabled                                */
    *stk-- = (INT8U)0x00;                   /* R0 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R1 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R20 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R21 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R22 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R23 = 0x00                                               */
 	
    tmp    = (INT16U)pdata;
    *stk-- = (INT8U)tmp;                    /* Simulate call to function with argument                  */
    *stk-- = (INT8U)(tmp >> 8);				/* R24, R25 contains argument pointer pdata 				*/

    *stk-- = (INT8U)0x00;                   /* R26 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R27 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R30 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R31 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R2  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R3  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R4  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R5  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R6  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R7  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R8  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R9  = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R10 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R11 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R12 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R13 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R14 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R15 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R16 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R17 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R28 = 0x00                                               */
    *stk-- = (INT8U)0x00;                   /* R29 = 0x00                                               */
    return ((void *)stk);
}


/*
*********************************************************************************************************
*                               START HIGHEST PRIORITY TASK READY-TO-RUN
*
* Description : This function is called by OSStart() to start the highest priority task that was created
*               by your application before calling OSStart().
*
* Note(s)     : 1) The (data)stack frame is assumed to look as follows:
*
*                  OSTCBHighRdy->OSTCBStkPtr --> SPL of (return) stack pointer           (Low memory)
*                                                SPH of (return) stack pointer
*                                                Flags to load in status register
*                                                RAMPZ
*                                                R31
*                                                R30
*                                                R27
*                                                .
*                                                .
*                                                R0
*                                                PCH
*                                                PCL                                     (High memory)
*
*                  where the stack pointer points to the task start address.
*
*
*               2) OSStartHighRdy() MUST:
*                      a) Call OSTaskSwHook() then,
*                      b) Set OSRunning to TRUE,
*                      c) Switch to the highest priority task.
*********************************************************************************************************
*/

void OSStartHighRdy (void) {
#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
	OSTaskSwHook();
#endif
	OSRunning++;
	SP = (INT16U) OSTCBHighRdy->OSTCBStkPtr;
	POP_ALL();
}

/*
*********************************************************************************************************
*                                       TASK LEVEL CONTEXT SWITCH
*
* Description : This function is called when a task makes a higher priority task ready-to-run.
*
* Note(s)     : 1) Upon entry,
*                  OSTCBCur     points to the OS_TCB of the task to suspend
*                  OSTCBHighRdy points to the OS_TCB of the task to resume
*
*               2) The stack frame of the task to suspend looks as follows:
*
*                                       SP+0 --> LSB of task code address
*                                         +1     MSB of task code address                (High memory)
*
*               3) The saved context of the task to resume looks as follows:
*
*                  OSTCBHighRdy->OSTCBStkPtr --> SPL of (return) stack pointer           (Low memory)
*                                                SPH of (return) stack pointer
*                                                Flags to load in status register
*                                                RAMPZ
*                                                R31
*                                                R30
*                                                R27
*                                                .
*                                                .
*                                                R0
*                                                PCH
*                                                PCL                                     (High memory)
*********************************************************************************************************
*/

void OSCtxSw (void) {
	PUSH_ALL();
	OSTCBCur->OSTCBStkPtr = (OS_STK *) SP;
#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
	OSTaskSwHook();
#endif
	OSPrioCur = OSPrioHighRdy;
	OSTCBCur = OSTCBHighRdy;
	SP = (INT16U) OSTCBHighRdy->OSTCBStkPtr;
	POP_ALL();
}

/*
**********************************************************************************************************
*                                INTERRUPT LEVEL CONTEXT SWITCH
*
* Description : This function is called by OSIntExit() to perform a context switch to a task that has
*               been made ready-to-run by an ISR.
*
* Note(s)     : 1) Upon entry,
*                  OSTCBCur     points to the OS_TCB of the task to suspend
*                  OSTCBHighRdy points to the OS_TCB of the task to resume
*
*               2) The stack frame of the task to suspend looks as follows:
*
*                  OSTCBCur->OSTCBStkPtr ------> SPL of (return) stack pointer           (Low memory)
*                                                SPH of (return) stack pointer
*                                                Flags to load in status register
*                                                RAMPZ
*                                                R31
*                                                R30
*                                                R27
*                                                .
*                                                .
*                                                R0
*                                                PCH
*                                                PCL                                     (High memory)
*
*               3) The saved context of the task to resume looks as follows:
*
*                  OSTCBHighRdy->OSTCBStkPtr --> SPL of (return) stack pointer           (Low memory)
*                                                SPH of (return) stack pointer
*                                                Flags to load in status register
*                                                RAMPZ
*                                                R31
*                                                R30
*                                                R27
*                                                .
*                                                .
*                                                R0
*                                                PCH
*                                                PCL                                     (High memory)
**********************************************************************************************************
*/

void OSIntCtxSw (void) {
#if OS_CRITICAL_METHOD == 1
	OSTCBCur->OSTCBStkPtr = (OS_STK *) SP+4;
#endif
#if OS_CRITICAL_METHOD == 2
	OSTCBCur->OSTCBStkPtr = (OS_STK *) SP+6;
#endif
#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
	OSTaskSwHook();
#endif
	OSPrioCur = OSPrioHighRdy;
	OSTCBCur = OSTCBHighRdy;
	SP = (INT16U) OSTCBHighRdy->OSTCBStkPtr;
	POP_ALL();
}

/*
*********************************************************************************************************
*                                           SYSTEM TICK ISR
*
* Description : This function is the ISR used to notify uC/OS-II that a system tick has occurred.
*
*               The following C-like pseudo-code describe the operation being performed in the code below.
*
*               Save all registers on the current task's stack:
*                      Use the PUSH_ALL macro
*                      Get the SREG, set Bit #7 and save onto the task's stack using -Y addressing
*                      Use the PUSH_SP macro to save the task's hardware stack pointer onto the current task's stack
*               OSIntNesting++;
*               if (OSIntNesting == 1) {
*                  OSTCBCur->OSTCBStkPtr = SP
*               }
*               Clear the interrupt;                  Not needed for the timer we used.
*               OSTimeTick();                         Notify uC/OS-II that a tick has occured
*               OSIntExit();                          Notify uC/OS-II about end of ISR
*               Restore all registers that were save on the current task's stack:
*                      Use the POP_SP macro to restore the task's hardware stack pointer
*                      Use the POP_SREG macro to restore the SREG register (but clear the interrupt mask)
*                      Use the POP_ALL macro to restore the remaining registers
*               Return from Interrupt
*********************************************************************************************************
*/

SIGNAL (SIG_OUTPUT_COMPARE2) {
	OS_INT_ENTER();
	OSTimeTick();
	OS_INT_EXIT();
}


void InitOSTimer(void) {

    TIMSK &= 0x3F;         // Disable TIMER2 Interrupts
    TCCR2  = 0x0D;
    TCNT2  =    0;         // Start TCNT at 0 for a new cycle
    OCR2   =  124;

    TIFR   |= 0xC0;         // Clear  TIMER2 compare Interrupt Flag
    TIMSK  |= 0x80;         // Enable TIMER2 compare Interrupt
}


/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                            (BEGINNING)
*
* Description: This function is called by OSInit() at the beginning of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
//#if OS_CPU_HOOKS_EN > 0 && OS_VERSION > 203
void  OSInitHookBegin (void)
{
}
//#endif

/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                               (END)
*
* Description: This function is called by OSInit() at the end of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
//#if OS_CPU_HOOKS_EN > 0 && OS_VERSION > 203
void  OSInitHookEnd (void)
{
}
//#endif

/*$PAGE*/
/*
*********************************************************************************************************
*                                          TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
//#if OS_CPU_HOOKS_EN > 0 
void  OSTaskCreateHook (OS_TCB *ptcb)
{
    ptcb = ptcb;                       /* Prevent compiler warning                                     */
}
//#endif


/*
*********************************************************************************************************
*                                           TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
#if OS_CPU_HOOKS_EN > 0 
void  OSTaskDelHook (OS_TCB *ptcb)
{
    ptcb = ptcb;                       /* Prevent compiler warning                                     */
}
#endif

/*
*********************************************************************************************************
*                                             IDLE TASK HOOK
*
* Description: This function is called by the idle task.  This hook has been added to allow you to do  
*              such things as STOP the CPU to conserve power.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are enabled during this call.
*********************************************************************************************************
*/
//#if OS_CPU_HOOKS_EN > 0 && OS_VERSION >= 251
void  OSTaskIdleHook (void)
{
}
//#endif

/*
*********************************************************************************************************
*                                           STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-II's statistics task.  This allows your 
*              application to add functionality to the statistics task.
*
* Arguments  : none
*********************************************************************************************************
*/

#if OS_CPU_HOOKS_EN > 0 
void  OSTaskStatHook (void)
{
}
#endif

/*
*********************************************************************************************************
*                                           TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other
*              operations during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                 will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the 
*                 task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
void  OSTaskSwHook (void)
{
}
#endif

/*
*********************************************************************************************************
*                                           OS_TCBInit() HOOK
*
* Description: This function is called by OS_TCBInit() after setting up most of the TCB.
*
* Arguments  : ptcb    is a pointer to the TCB of the task being created.
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
//#if OS_CPU_HOOKS_EN > 0 && OS_VERSION > 203
void  OSTCBInitHook (OS_TCB *ptcb)
{
    ptcb = ptcb;                                           /* Prevent Compiler warning                 */
}
//#endif


/*
*********************************************************************************************************
*                                               TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_TIME_TICK_HOOK_EN > 0)
void  OSTimeTickHook (void)
{
}
#endif

