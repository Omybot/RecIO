#include <p33FJ128MC804.h>
#include "init.h"
#include "asser.h"
#include "Pilotage.h"

#define LIMITATION_COURANT 600 //800

extern unsigned char pid_power[2];
extern unsigned char securite_ascenseur;
unsigned int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int ADC_Results[9],DmaBuffer = 0,cpt_moy=0,courant=0;
float sum;
unsigned char telem,telem_front;

void InitClk(void)
{	
	PLLFBD = 38;				// Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
	CLKDIV = 0x0000;			// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2
	
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(OSCCON | 0x01);
	while(OSCCONbits.COSC != 0b011);
	while(OSCCONbits.LOCK != 1);
}

void Init_Interrupt_Priority(void)
{

	IPC0bits.T1IP   = 4;	//5		// Timer 1 used by Ethernet (Default value = 2)
	IPC1bits.T2IP   = 5;	//6		// Timer 2 used to generate PWM
	IPC2bits.T3IP   = 7;			// Timer 3 used by Input Capture (IC1)
	IPC2bits.U1RXIP = 4;			// UART RX Interrupt
	IPC3bits.U1TXIP = 4;			// UART TX Interrupt
	IPC6bits.T4IP   = 3;			// Timer 4 Used by Asser
	IPC7bits.T5IP   = 6;	//5		// Timer 5 Used by Canon
	IPC14bits.QEI1IP = 7;			// Quad Encoder Interrupt
	IPC18bits.QEI2IP = 7;			// Quad Encoder Interrupt
	IPC15bits.DMA5IP = 6;			// DMA5 Interrupt
	IPC0bits.IC1IP   = 6;			// Input Capture used by Capteur Couleur
}

void InitPorts(void) 
{	
	AD1PCFGL=0xFFFF;	//Tous les ports Analogiques configurés en numérique
	
	TRISAbits.TRISA0=1; // MOT4_I - Moteurs
	TRISAbits.TRISA1=1; // MOT2_I - Moteurs
	TRISAbits.TRISA2=1; // OSC - Oscillateur
	TRISAbits.TRISA3=0; // RA3 - Mezzanine - (RECIO)AX12/CDS Direction
	TRISAbits.TRISA4=0; // RST - Ethernet RST
	TRISAbits.TRISA7=0; // RA7 - Mezzanine - (RECIO)Capteur couleur - S3
	TRISAbits.TRISA8=0; // RA8 - Mezzanine - (RECIO)Capteur couleur - LED
	TRISAbits.TRISA9=0; // CS - Ethernet CS
	TRISAbits.TRISA10=0; // RA10 - Mezzanine - (RECIO)Capteur couleur - S2
	
	TRISBbits.TRISB0=1; // MOT1_I - Moteurs
	TRISBbits.TRISB1=1; // MOT3_I - Moteurs
	TRISBbits.TRISB2=1; // RB2 - Mezzanine - (RECIO)Disponible
	TRISBbits.TRISB3=1; // RB3 - Mezzanine - (RECIO)Lulu TX - UART RX
	TRISBbits.TRISB4=1; // RB4 - Mezzanine - (RECIO)Capteur couleur - OUT
	TRISBbits.TRISB5=1; // PGD - JTAG 
	TRISBbits.TRISB6=1; // PGC - JTAG 
	TRISBbits.TRISB7=1; // INT - Ethernet INT
	TRISBbits.TRISB8=1; // CODEUR1B - Codeurs
	TRISBbits.TRISB9=1; // CODEUR1A - Codeurs
	TRISBbits.TRISB10=0; // MOT2_PWMH - Moteurs
	TRISBbits.TRISB11=0; // MOT2_PWML - Moteurs
	TRISBbits.TRISB12=0; // MOT3_PWMH - Moteurs
	TRISBbits.TRISB13=0; // MOT3_PWML - Moteurs
	TRISBbits.TRISB14=0; // MOT4_PWMH - Moteurs
	TRISBbits.TRISB15=0; // MOT4_PWML - Moteurs
	
	TRISCbits.TRISC0=0; // RC0 - Mezzanine - (RECIO)Lulu RX - UART TX
	TRISCbits.TRISC1=1; // RC1 - Mezzanine - (RECIO)AX12/CDS RX
	TRISCbits.TRISC2=0; // RC2 - Mezzanine - (RECIO)AX12/CDS TX
	TRISCbits.TRISC3=0; // SCK1 - Ethernet MSCK
	TRISCbits.TRISC4=0; // SDO1 - Ethernet MOSI
	TRISCbits.TRISC5=1; // SDI1 - Ethernet MISO
	TRISCbits.TRISC6=0; // MOT1_PWMH - Moteurs
	TRISCbits.TRISC7=0; // MOT1_PWML - Moteurs
	TRISCbits.TRISC8=1; // CODEUR2A - Codeurs
	TRISCbits.TRISC9=1; // CODEUR2B - Codeurs

	//Configuration des ports pour la liaison SPI avec le module Ethernet
	RPOR9bits.RP19R   = 0b01000; // SCK1  		<==> RP19 RC3 
	RPOR10bits.RP20R  = 0b00111; // SDO1		<==> RP20 RC4
	RPINR20bits.SDI1R = 21     ; // SDI1 		<==> RP21 RC5
	
	// UART AX12/CDS
	RPOR9bits.RP18R = 0b00011;		//TX RP18 U1TX
    RPINR18bits.U1RXR = 17;			//RX RP17 U1RXR
	LATAbits.LATA3 = 1;	// 1 J'envoie et 0 je réceptionne	//Initialisation du sens de communication pour les AX12
    
	// UART Lulu
	RPOR8bits.RP16R = 0b00101;		//TX RP16 U2TX
    RPINR19bits.U2RXR = 3;			//RX RP3 U2RXR

	// Capteur de couleur OUT  
	RPINR7bits.IC1R 	= 4;		//RP4
}


void Init_Input_Capture(void)
{
	// Use Timer 3 for IC1
	Init_Timer3();

	// Set IC1 to Capture
	IC1CONbits.ICM		= 0;		// Disable Input Capture 1 Module
	IC1CONbits.ICTMR	= 0;		// Select Timer 3 as the time base
	IC1CONbits.ICI		= 0b01;		// Interrupt on every 2nd capture event
	IC1CONbits.ICM		= 0b101;	// Capture mode, every 16th rising edge	

	IFS0bits.IC1IF = 0;
	IEC0bits.IC1IE = 1;
}


void Init_Timer(void)
{
	Init_Timer4();
}


void Init_Timer2(void)		
{
	T2CONbits.TCKPS = 0b11;	// 1:256 Prescaler
	PR2 = 0xFFFF;			// Time to autoreload
	IFS0bits.T2IF = 0;		// Interrupt flag cleared
	IEC0bits.T2IE = 0;		// Interrupt disabled
	T2CONbits.TON = 1;		// Timer enabled
}

void Init_Timer3(void)
{
//	T3CONbits.TON 	= 0;	//Stops the timer
//	T3CONbits.TSIDL = 0;
//	T3CONbits.TGATE = 0;
//	T3CONbits.TCS	= 0;
	T3CONbits.TCKPS = 0b01; //Prescaler set to 1:8

	IFS0bits.T3IF = 0; 		//Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 0; 		//Disable Timer3 interrupt	
	
	T3CONbits.TON = 1;		//Starts the timer
}

void Init_Timer4(void)
{
	//--Timer4
	T4CONbits.TON 	= 0;	//Stops the timer
	T4CONbits.TSIDL = 0;
	T4CONbits.TGATE = 0;
	T4CONbits.TCS	= 0;
	T4CONbits.TCKPS = 0b01; //Prescaler set to 1:8
	
	TMR4 = 0; 				//Clear timer register
	PR4  = 5000; 			//Load the period value (Pas)

	IFS1bits.T4IF = 0; 		//Clear Timer4 Interrupt Flag
	IEC1bits.T4IE = 1; 		//Enable Timer4 interrupt
	
	T4CONbits.TON = 1;		//Starts the timer
}

void Init_Timer5(void)
{
	//--Timer5
	T5CONbits.TON 	= 0;	//Stops the timer
	T5CONbits.TSIDL = 0;
	T5CONbits.TGATE = 0;
	T5CONbits.TCS	= 0;
	T5CONbits.TCKPS = 0b00; //Prescaler set to 1:1
	
	TMR5 = 0; 				//Clear timer register
	PR5  = 6000;     		//Load the period value (Pas) 

	IFS1bits.T5IF = 0; 		//Clear Timer5 Interrupt Flag
	IEC1bits.T5IE = 1; 		//Enable Timer5 interrupt
}

void InitPWM(void)
{
	PWM1CON2bits.OSYNC = 1;
	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output
	PWM1CON1bits.PEN1H = 0;		// PWM1H1 pin is disabled for PWM output
	PWM1CON1bits.PEN2L = 1;		// PWM1L2 pin is enabled for PWM output
	PWM1CON1bits.PEN2H = 0;		// PWM1H2 pin is disabled for PWM output
	PWM1CON1bits.PEN3L = 0;		// PWM1L3 pin is disabled for PWM output
	PWM1CON1bits.PEN3H = 1;		// PWM1H3 pin is enabled for PWM output
	
	PWM2CON2bits.OSYNC = 1;
	P2TCONbits.PTEN = 1; 		// PWM Time base is On
	P2TPER = 2000 - 1; 			// 20kHz PWM (2000 counts @40MIPS)
	PWM2CON1bits.PEN1L = 1;		// PWM1L1 pin is enabled for PWM output
	PWM2CON1bits.PEN1H = 0;		// PWM1H1 pin is disabled for PWM output
	

//	DIR_ASCENSEUR_GAUCHE	= 0;
//	DIR_ASCENSEUR_DROITE	= 0;
//	LAT_ASCENSEUR_GAUCHE 	= 0;
//	LAT_ASCENSEUR_DROITE	= 0;
//	LAT_POMPE	= 0;
//
//	PWM_ASCENSEUR_GAUCHE	= 0xFFFF;				// 0x0000 = 100.00% Power
//	PWM_ASCENSEUR_DROITE	= 0xFFFF;				// 0xFFFF =   0.00% Power
//	PWM_BALISE				= 0xFFFF;
//	PWM_POMPE				= 0xFFFF;
}

void InitQEI(void)
{
	QEI1CONbits.QEIM  = 0b111;
	QEI2CONbits.QEIM  = 0b111;
	QEI1CONbits.SWPAB = 1;
	QEI2CONbits.SWPAB = 0;
	POS1CNT = 0x0000;
	POS2CNT = 0x0000;
	//DFLT1CONbits.QECK = 0b111;
	//DFLT2CONbits.QECK = 0b111;
	IFS3bits.QEI1IF = 0;
	IFS4bits.QEI2IF = 0;
	IEC3bits.QEI1IE = 1;
	IEC4bits.QEI2IE = 1;
}

void InitADC(void)
{
	AD1CON1bits.FORM   = 0;		// Data Output Format: Integer
	AD1CON1bits.SSRC   = 7;		// Sample Clock Source: Conversion autostart
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 1;		// 12-bit ADC operation

	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
	AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us	
	
	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);	// 6 ADC Channel is scanned ???
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
	AD1CSSLbits.CSS0=1;		// Enable AN0 for channel scan
	AD1CSSLbits.CSS1=1;		// Enable AN1 for channel scan
	AD1CSSLbits.CSS2=1;		// Enable AN2 for channel scan
	AD1CSSLbits.CSS3=1;		// Disable AN3 for channel scan
	AD1CSSLbits.CSS4=1;		// Disable AN4 for channel scan
	AD1CSSLbits.CSS5=1;		// Enable AN5 for channel scan
	AD1CSSLbits.CSS6=1;		// Enable AN6 for channel scan
	AD1CSSLbits.CSS7=1;		// Enable AN7 for channel scan
	AD1CSSLbits.CSS8=1;		// Disable AN8 for channel scan
	
 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;
	AD1PCFGLbits.PCFG0 = 0;	// AN0 as Analog Input
	AD1PCFGLbits.PCFG1 = 0;	// AN1 as Analog Input
 	AD1PCFGLbits.PCFG2 = 0;	// AN2 as Analog Input 
	AD1PCFGLbits.PCFG3 = 0;	// AN3 as Analog Input 
	AD1PCFGLbits.PCFG4 = 0;	// AN4 as Analog Input  
	AD1PCFGLbits.PCFG5 = 1;	// AN5 as Digital Input  
	AD1PCFGLbits.PCFG6 = 1;	// AN6 as Digital Input 
	AD1PCFGLbits.PCFG7 = 1;	// AN7 as Digital Input 
	AD1PCFGLbits.PCFG8 = 1;	// AN8 as Digital Input 
	
	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt 
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}

void InitDMA(void)
{
	
	DMA5CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
	DMA5CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA5PAD=(int)&ADC1BUF0;
	DMA5CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
	DMA5REQ = 13;					// Select ADC1 as DMA Request source

	DMA5STA = __builtin_dmaoffset(BufferA);		
	DMA5STB = __builtin_dmaoffset(BufferB);

	IFS3bits.DMA5IF = 0; //Clear the DMA interrupt flag bit
	IEC3bits.DMA5IE = 1; //Set the DMA interrupt enable bit
	
	DMA5CONbits.CHEN=1;				// Enable DMA
}

void InitUART(void)
{
	InitUART1();
	InitUART2();
}

void InitUART1(void) 
{
    U1BRG = 522;				// ? Baud
	U1MODEbits.UARTEN = 1;		// UART1 is Enabled
	U1MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U1MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U1MODEbits.RTSMD = 0; 		// flow control mode
	U1MODEbits.UEN = 0b00;		// UTX, RTX, U2CTS, U2RTS are enable and on use.
	U1MODEbits.WAKE = 0;		// Wake-up on start bit is enabled
	U1MODEbits.LPBACK = 0;		// Loop-back is disabled
	U1MODEbits.ABAUD = 0;		// auto baud is disabled
	U1MODEbits.URXINV = 0;		// No RX inversion
	U1MODEbits.BRGH = 1;		// low boud rate
	U1MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U1MODEbits.STSEL = 0;		// one stop bit

	U1STAbits.UTXISEL1 = 0b00;
	U1STA &= 0xDFFF;			// clear TXINV by bit masking
	U1STAbits.UTXBRK = 0;		// sync break tx is disabled
	U1STAbits.UTXEN = 1;		// transmit  is enabled
	U1STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled whith 1 character
	U1STAbits.ADDEN = 0;		// address detect mode is disabled

	IFS0bits.U1TXIF = 0;         // clear UART Tx interrupt flag
	IEC0bits.U1TXIE = 0;         // enable UART Tx interrupt

	IFS0bits.U1RXIF = 0;		 // clear interrupt flag of rx
	IEC0bits.U1RXIE = 1;		 // enable rx recieved data
}

void InitUART2(void)
{
	U2BRG = 259;				// 115200 baud
	U2MODEbits.UARTEN = 1;		// UART2 is Enabled
	U2MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U2MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U2MODEbits.RTSMD = 0; 		// flow control mode
	U2MODEbits.UEN = 0b00;		// UTX, RTX, are enable and on use.
	U2MODEbits.WAKE = 0;		// Wake-up on start bit is enabled
	U2MODEbits.LPBACK = 0;		// Loop-back is disabled
	U2MODEbits.ABAUD = 0;		// auto baud is disabled
	U2MODEbits.URXINV = 0;		// No RX inversion
	U2MODEbits.BRGH = 0;		// low boud rate
	U2MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U2MODEbits.STSEL = 0;		// one stop bit

	U2STAbits.UTXISEL1 = 0b00;
	U2STA &= 0xDFFF;			// clear TXINV by bit masking
	U2STAbits.UTXBRK = 0;		// sync break tx is disabled
	U2STAbits.UTXEN = 1;		// transmit  is enabled
	U2STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled whith 1 character
	U2STAbits.ADDEN = 0;		// address detect mode is disabled

	IFS1bits.U2TXIF = 0;         // clear UART Tx interrupt flag
	IEC1bits.U2TXIE = 0;         // enable UART Tx interrupt

	IFS1bits.U2RXIF = 0;		 // clear interrupt flag of rx
	IEC1bits.U2RXIE = 1;		 // enable rx recieved data
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
{
	static int seuil[2]={2500,2500};
	static int hysteresis[2]={50,50};
	unsigned int telemetre[2];
	static unsigned char i,cpt_secu_courant[2]={0,0};
	ADC_Results[0]=0;
	ADC_Results[1]=0;
	ADC_Results[2]=0;
	ADC_Results[3]=0;
	ADC_Results[4]=0;
	ADC_Results[5]=0;
	ADC_Results[6]=0;
	ADC_Results[7]=0;
	ADC_Results[8]=0;
	if(DmaBuffer == 0)
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferA[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferA[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferA[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferA[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferA[4][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferA[5][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[6] += BufferA[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[7] += BufferA[7][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[8] += BufferA[8][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
		ADC_Results[6] /= SAMP_BUFF_SIZE;
		ADC_Results[7] /= SAMP_BUFF_SIZE;
		ADC_Results[8] /= SAMP_BUFF_SIZE;
	}
	else
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferB[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferB[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferB[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferB[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferB[4][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferB[5][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[6] += BufferB[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[7] += BufferB[7][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[8] += BufferB[8][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
		ADC_Results[6] /= SAMP_BUFF_SIZE;
		ADC_Results[7] /= SAMP_BUFF_SIZE;
		ADC_Results[8] /= SAMP_BUFF_SIZE;
	}

	DmaBuffer ^= 1;

	IFS3bits.DMA5IF = 0;		// Clear the DMA0 Interrupt Flag
}
