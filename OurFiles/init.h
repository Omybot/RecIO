#ifndef __INIT_H__
#define __INIT_H__

#define MATCH_COULEUR	PORTAbits.RA8
#define MATCH_JACK		PORTBbits.RB4

#define COEFF_TENSION_ADC 0.0008056640625

void InitClk(void);
void InitPorts(void);
void InitPWM(void); 
void InitQEI(void);

void Init_Timer2(void);
void Init_Timer5(void);
void Init_Timer4(void);

void InitADC(void);
void InitDMA(void);
void Init_Timer(void);
void Init_Input_Capture(void);
void Init_Interrupt_Priority(void);

//#define  MAX_CHNUM	 			7		// Highest Analog input number in Channel Scan
//#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
//#define  NUM_CHS2SCAN			8		// Number of channels enabled for channel scan
#define  MAX_CHNUM	 			8		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#define  NUM_CHS2SCAN			9		// Number of channels enabled for channel scan

#endif // __INIT_H__
