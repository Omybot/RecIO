#include "Stepper.h"

//Variable Stepper
unsigned int cpt_tmr5_step = 0;
unsigned int nb_step = 0;
unsigned int posActuelle = 0;


void InitSwitchFinDeCourse(void)
{
	CNEN2bits.CN29IE  	  = 1;		// Pin 31 - RA3 
	IEC1bits.CNIE 	  	  = 0; 		// (Actif uniquement sur demande prise d'origine)
	IFS1bits.CNIF 	  	  = 0;		// Reset CN interrupt
	//CNPU2bits.CN29PUE = 1;		// Switch fin de course (deja declare sur init.c)
}

void InitStepper(void)
{
	W_CLK_STEPPER = 0;
//	W_DIR_STEPPER = DIR_DOWN;
	nb_step = 0;

	Init_Timer5();
	InitSwitchFinDeCourse();
}
//Recalage en Z
void InitPosStepper(void)
{
	IEC1bits.CNIE = 1;
	if(R_SWITCH_ARR == 0)
		return;
	nb_step = INFINI;			//Step infini 
//	W_DIR_STEPPER 	= DIR_DOWN;	//Descente vers le switch (0 = DOWN; 1 = UP)
	T5CONbits.TON 	= 1;		//Start timer
}

void SetStepperPosition(unsigned int pos)
{
	if(pos == posActuelle)
		return;
	else if(pos > posActuelle)
	{
//		W_DIR_STEPPER = DIR_UP;
		nb_step = pos - posActuelle;	
	}
	else if(pos < posActuelle)
	{
//		W_DIR_STEPPER = DIR_DOWN;
		nb_step = posActuelle - pos;
	}
	posActuelle = pos;	
	nb_step = nb_step*2; 				//Step every rising edge
	
	//Start timer
	T5CONbits.TON 	= 1;
}
// Interrupt every 100us
void __attribute__ ((interrupt, no_auto_psv)) _T5Interrupt(void) 
{	
	if(nb_step != INFINI)
	{
		nb_step--;
		/*if(R_DIR_STEPPER == DIR_UP)
			posActuelle++;
		if(R_DIR_STEPPER == DIR_DOWN)
			posActuelle--;*/
		if(nb_step == 0)
			T5CONbits.TON = 0;			//Stop timer
	}
	W_CLK_STEPPER = !R_CLK_STEPPER;
	IFS1bits.T5IF = 0;	
}

void __attribute__ ((interrupt, no_auto_psv)) _CNInterrupt(void)
{
	if(R_SWITCH_ARR == 0)
	{
		IEC1bits.CNIE = 0;
		T5CONbits.TON 	= 0;			// Stop timer
		posActuelle 	= 0;
//		W_DIR_STEPPER = DIR_UP; 		// Inversion de sens
	}
	IFS1bits.CNIF = 0; 					// Clear CN Interrupt
}
