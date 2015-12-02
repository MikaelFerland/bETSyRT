/*
*********************************************************************************************************
*                                               uC/OS-II
*                                        The Real-Time Kernel
*
*                                          ATMega32 Specific code
*
* File         : INCLUDES.H
* By           : Bruno De Kelper
* Port Version : V1.0
*********************************************************************************************************
*/
#ifndef INCLUDE_H
#define INCLUDE_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h> 
#include <string.h> 

#include  "os_cpu.h"
#include  "os_cfg.h"
#include  "ucos_ii.h"

#include "global_vars.h"
#include "uart.h"
#include "timer.h"
#include "moteur.h"
#include "adc.h"
#include "i2c.h"

#endif
