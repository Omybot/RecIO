#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "init.h"
#include <p33FJ128MC804.h>

//#define W_DIR_STEPPER LATBbits.LATB12
#define W_CLK_STEPPER LATBbits.LATB13
#define R_DIR_STEPPER PORTBbits.RB12
#define R_CLK_STEPPER PORTBbits.RB13
#define R_SWITCH_ARR  PORTAbits.RA3
#define INFINI -1
#define DIR_UP 	 1
#define DIR_DOWN 0

void InitStepper(void);
void InitPosStepper(void);
void InitSwitchFinDeCourse(void);
void SetStepperPosition(unsigned int pos);

#endif
