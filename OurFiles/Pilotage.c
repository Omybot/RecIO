#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>
#include "CDS5516.h"
#include "init.h"
#include "Stepper.h"

#define UART_BUFFER_SIZE 100
#define UART_BUFFER_SIZE2 100

// Define Capteur Couleur
#define LED LATAbits.LATA8
#define S2 LATAbits.LATA10
#define S3 LATAbits.LATA7

extern int revolutions[N];
//	Extern Capteur Couleur
extern unsigned int Tab_Capteur_Couleur[8];
extern unsigned char alim_capteur_couleur;
// UART
extern 	unsigned char flag_envoi_uart,buffer_envoi_uart[UART_BUFFER_SIZE],ptr_write_buffer_uart;
extern 	unsigned char flag_envoi_uart2,buffer_envoi_uart2[UART_BUFFER_SIZE2],ptr_write_buffer_uart2;

// ATTENTION /!\ Ces fonctions ne doivent pas �tre bloquantes
extern unsigned int position_ascenseur_gauche,position_ascenseur_droite,courant;
extern unsigned char jackAvant,tir,tir_power;
extern unsigned int capteur_vitesse;
extern unsigned char desactive_interrupt;
extern unsigned int cpt_capteur_vitesse;

extern unsigned int cpt_20mscourant;

extern unsigned int ADC_Results[9];
extern unsigned int position_buffer[6];
extern unsigned char buff_position_ptr,last_send_ptr;
extern long buff_position[N][256];
extern unsigned char buff_status_ptr,last_send_status_ptr;
extern unsigned int buff_status[3][64];
extern long raw_position[2];
extern double bridage;
extern double erreur[N];
extern double targ_pos[N];
extern double real_pos[N];
extern double cons_pos[N];
extern double etape_pos[5];
extern double pwm_cor[N];
extern double feedforward;
extern unsigned int PID_ressource_used,electrovanne;
extern double xydistance;
extern double xyangle;
extern double position_lock;
extern double pos_x;
extern double pos_y;
extern double pos_teta;
extern double offset_teta;

extern double erreur_allowed;
extern unsigned char flag_capteur_vitesse;

unsigned char scan;

//Variable Capteur Couleur
unsigned int Valeur_Capteur_Couleur = 24;
unsigned int Cpt_Tmr_Periode = 0;

int PiloteStop(unsigned char id_moteur,unsigned char stopmode)
{
	Motors_Stop(stopmode,id_moteur);		
	return 1;
}

void delay(void)
{
    long i = 10;
    while(i--);
}
void delay10ms(void)
{
    long i = 40000;
    while(i--);
}
void delayms(void)
{
	long i = 1600000; //400ms
    while(i--);
}
//Delay seconde
void delays(void)
{
	long i = 4000000; //seconde
    while(i--);
}

//Interruption Input Capture
//Interruption induced on every 16th Rising Edge
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void)
{
	unsigned int t1,t2;
	t1=IC1BUF;
	t2=IC1BUF;

	IFS0bits.IC1IF=0;

	if(t2>t1)
		Valeur_Capteur_Couleur = t2-t1;
	else
		Valeur_Capteur_Couleur = (PR3 - t1) + t2;
}

Trame Retour_Capteur_Onoff(unsigned char id_capteur)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[4];
	Etat_Valeurs.nbChar = 4;
	
	Valeurs[0] = UDP_ID;
	Valeurs[1] = CMD_REPONSE_CAPTEUR_ONOFF;
	Valeurs[2] = id_capteur;
	switch(id_capteur)
	{
		case VACUOSTAT_BACK:
			Valeurs[3] = !PORTCbits.RC1;
			break;
		case VACUOSTAT_FRONT:
			Valeurs[3] = PORTCbits.RC2;
			break;
	}
	
	
	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

Trame Retour_Valeurs_Analogiques(void)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[4];
	Etat_Valeurs.nbChar = 20;


	Valeurs[0] = 0xC4;
	Valeurs[1] = CMD_REPONSE_VALEURS_ANALOGIQUES;
	Valeurs[2] = ADC_Results[0] >> 8;
	Valeurs[3] = ADC_Results[0] & 0xFF;
	Valeurs[4] = ADC_Results[1] >> 8;
	Valeurs[5] = ADC_Results[1] & 0xFF;
	Valeurs[6] = ADC_Results[2] >> 8;
	Valeurs[7] = ADC_Results[2] & 0xFF;
	Valeurs[8] = ADC_Results[3] >> 8;
	Valeurs[9] = ADC_Results[3] & 0xFF;
	Valeurs[10] = ADC_Results[4] >> 8;
	Valeurs[11] = ADC_Results[4] & 0xFF;
	Valeurs[12] = ADC_Results[5] >> 8;
	Valeurs[13] = ADC_Results[5] & 0xFF;
	Valeurs[14] = ADC_Results[6] >> 8;
	Valeurs[15] = ADC_Results[6] & 0xFF;
	Valeurs[16] = ADC_Results[7] >> 8;
	Valeurs[17] = ADC_Results[7] & 0xFF;
	Valeurs[18] = ADC_Results[8] >> 8;
	Valeurs[19] = ADC_Results[8] & 0xFF;

	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

unsigned int Send_Variable_Capteur_Couleur(void){
	return Valeur_Capteur_Couleur;
}

Trame Couleur_Balle(void)
{

	Trame Couleur_Balle;
	static BYTE Couleur[18];
	Couleur_Balle.nbChar = 18;

	Couleur[0] = 0xC4;
	Couleur[1] = CMD_DEBUG; //CMD_REPONSE_COULEUR_BALLE

	Couleur[2] = Tab_Capteur_Couleur[0]>>8;
	Couleur[3] = Tab_Capteur_Couleur[0]&0x00FF;

	Couleur[4] = Tab_Capteur_Couleur[1]>>8;
	Couleur[5] = Tab_Capteur_Couleur[1]&0x00FF;

	Couleur[6] = Tab_Capteur_Couleur[2]>>8;
	Couleur[7] = Tab_Capteur_Couleur[2]&0x00FF;

	Couleur[8] = Tab_Capteur_Couleur[3]>>8;
	Couleur[9] = Tab_Capteur_Couleur[3]&0x00FF;

	Couleur[10] = Tab_Capteur_Couleur[4]>>8;
	Couleur[11] = Tab_Capteur_Couleur[4]&0x00FF;

	Couleur[12] = Tab_Capteur_Couleur[5]>>8;
	Couleur[13] = Tab_Capteur_Couleur[5]&0x00FF;

	Couleur[14] = Tab_Capteur_Couleur[6]>>8;
	Couleur[15] = Tab_Capteur_Couleur[6]&0x00FF;

	Couleur[16] = Tab_Capteur_Couleur[7]>>8;
	Couleur[17] = Tab_Capteur_Couleur[7]&0x00FF;

	Couleur_Balle.message = Couleur;

	return Couleur_Balle;

}

Trame CouleurRGB(int Id)
{
	Rgb RgbVal;
	Trame RgbMessage;	
	double freqClear,freqRed,freqGreen,freqBlue;	
	static BYTE Couleur[6];
	
	RgbMessage.nbChar = 6;
	Couleur[0] = 0xC4;
	Couleur[1] = CMD_REPONSE_CAPTEUR_COULEUR;
	Couleur[2] = Id;

	freqClear = period2frequency(Tab_Capteur_Couleur[6]);
	freqRed   = period2frequency(Tab_Capteur_Couleur[4]);
	freqGreen = period2frequency(Tab_Capteur_Couleur[7]);
	freqBlue  = period2frequency(Tab_Capteur_Couleur[5]);

	RgbVal = frequency2RGB(freqClear, freqRed, freqGreen, freqBlue);

	Couleur[3] = RgbVal.red;
	Couleur[4] = RgbVal.green;
	Couleur[5] = RgbVal.blue;

	RgbMessage.message = Couleur;
	return RgbMessage;
}

double period2frequency(unsigned int period)
{
	double frequency;
	double fcy = 40000000;   // 40MHz
	BYTE timerPrescaler = 8; // Timer TCKPS prescaler value
	BYTE captureEventConfig = 16; // Event captured on every 16th rising edge
	double timeBaseFreq = fcy/timerPrescaler;
	double timeBasePeriod = 1/(timeBaseFreq);   // period of 1 tick of the time base
	
	// frequency of the timer value captured by IC mdoule
	frequency = 1/((period/captureEventConfig)*timeBasePeriod);
	return frequency;
}

Rgb frequency2RGB(double freqClear, double freqRed, double freqGreen, double freqBlue)
{
	Rgb rgbVal;
	rgbVal.red = (freqRed/freqClear)*255;
	rgbVal.green = (freqGreen/freqClear)*255;
	rgbVal.blue = (freqBlue/freqClear)*255;
	
	return rgbVal;  
}

Trame StatusMonitor(void)
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;

	tableau[0] = 0xC4; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_STATUS;

	if(buff_status_ptr > last_send_status_ptr)
		nbr_to_send = buff_status_ptr - last_send_status_ptr;
	else
		nbr_to_send = 64 - last_send_status_ptr + buff_status_ptr;

	//nbr_to_send = (buff_status_ptr - last_send_status_ptr)%64;


	last_send_status_ptr=buff_status_ptr;
	if(nbr_to_send>35) nbr_to_send=35;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*6+4;

	current_send_ptr = last_send_status_ptr + 1;

	for(i=0;i<nbr_to_send;i++)
	{
		tableau[1+2+(i*6)] = buff_status[0][current_send_ptr]>>8; // Status
		tableau[1+3+(i*6)] = buff_status[0][current_send_ptr]&0x00FF;
		tableau[1+4+(i*6)] = buff_status[1][current_send_ptr]>>8; // PWM gauche
		tableau[1+5+(i*6)] = buff_status[1][current_send_ptr]&0x00FF;
		tableau[1+6+(i*6)] = buff_status[2][current_send_ptr]>>8; // PWM droite
		tableau[1+7+(i*6)] = buff_status[2][current_send_ptr]&0x00FF;
		current_send_ptr = (current_send_ptr + 1)%64;
	}

	trame.message = tableau;

	return trame;
}


void PilotePIDInit(void)
{
	InitProp();
}


Trame PilotePositionXYT()
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[8];
	trame.nbChar = 8;

	tableau[0] = 0xC4;
	tableau[1] = CMD_RETOURPOSITION;
	tableau[2] = (int)(pos_x * 10)>>8;
	tableau[3] = (int)(pos_x * 10)&0x00FF;
	tableau[4] = (int)(pos_y * 10)>>8;
	tableau[5] = (int)(pos_y * 10)&0x00FF;
	tableau[6] = (unsigned int)(pos_teta*36000/(2*PI)+18000)>>8;
	tableau[7] = (unsigned int)(pos_teta*36000/(2*PI)+18000)&0x00FF;

	trame.message = tableau;

	return trame;
}

Trame PilotePIDRessource()
{
	Trame trame;
	static BYTE tableau[4];
	trame.nbChar = 4;

	tableau[0] = 1;
	tableau[1] = 0x66;
	tableau[2] = PID_ressource_used>>8;
	tableau[3] = PID_ressource_used&0x00FF;

	trame.message = tableau;

	return trame;
}

void PilotePIDManual(unsigned int gauche,unsigned int droite)
{
	manual_pid((double)gauche,(double)droite);
}

void PilotePIDBridage(unsigned int value)
{
	bridage = (double)value;
}

void PilotePIDFeedforward(unsigned int value)
{
	feedforward = (double)value;
}

Trame PilotePIDErreurs()
{
	Trame trame;
	static BYTE tableau[10];
	trame.nbChar = 10;
	unsigned int data[4];

	tableau[0] = 1;
	tableau[1] = 0x48;

	data[0] = (unsigned int)fabs(pwm_cor[0]);
	data[1] = (unsigned int)fabs(pwm_cor[1]);
	data[2] = (unsigned int)fabs(real_pos[0]);
	data[3] = (unsigned int)fabs(real_pos[1]);


	tableau[2] = (int)pwm_cor[0]>>8;
	tableau[3] = (int)pwm_cor[0]&0x00FF;
	tableau[4] = (int)pwm_cor[1]>>8;
	tableau[5] = (int)pwm_cor[1]&0x00FF;
	tableau[6] = (int)raw_position[0]>>8;
	tableau[7] = (int)raw_position[0]&0x00FF;
	tableau[8] = (int)raw_position[1]>>8;
	tableau[9] = (int)raw_position[1]&0x00FF;

	trame.message = tableau;

	return trame;
}
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd)
{
	double coeffs[N*3]={DEFAULT_KP,DEFAULT_KI,DEFAULT_KD,DEFAULT_KP,DEFAULT_KI,DEFAULT_KD};
	coeffs[0] = (double)new_kp;
	coeffs[1] = (double)new_ki;
	coeffs[2] = (double)new_kd;
	coeffs[3] = (double)new_kp;
	coeffs[4] = (double)new_ki;
	coeffs[5] = (double)new_kd;

	set_pid(coeffs);
}


int Coupure(void)
{
	Stop(0); // FREELY
	return 0;
}

Trame PiloteGetPosition(unsigned char cote)
{
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;
	double position;

	tableau[0] = 1;
	tableau[1] = 0x42;
	tableau[2] = cote;

	switch(cote)
	{
		case 3:	position = Motors_GetPosition(0);
				break;
		case 4:	position = Motors_GetPosition(1);
				break;
		default:	break;
	}

	tableau[3] = ((unsigned int)fabs(position))>>8;
	tableau[4] = ((unsigned int)fabs(position))&0x00FF;
	if(position<0) 	tableau[5] = 1;
	else			tableau[5] = 0;
	trame.message = tableau;

	return trame;
}

Trame PiloteGetRawPosition()
{
	Trame trame;
	static BYTE tableau[15];
	trame.nbChar = 15;

	tableau[0] = 1;
	tableau[1] = CMD_DEMANDE_BUFF_POSITION;
	tableau[2] = 0;


	tableau[3] = position_buffer[0]>>8;
	tableau[4] = position_buffer[0]&0x00FF;
	tableau[5] = position_buffer[1]>>8;
	tableau[6] = position_buffer[1]&0x00FF;
	tableau[7] = position_buffer[2]>>8;
	tableau[8] = position_buffer[2]&0x00FF;
	tableau[9] = position_buffer[3]>>8;
	tableau[10] = position_buffer[3]&0x00FF;
	tableau[11] = position_buffer[4]>>8;
	tableau[12] = position_buffer[4]&0x00FF;
	tableau[13] = position_buffer[5]>>8;
	tableau[14] = position_buffer[5]&0x00FF;

	trame.message = tableau;

	return trame;
}

Trame PiloteGetLongPosition()
{
	Trame trame;
	static BYTE tableau[10];
	trame.nbChar = 10;

	tableau[0] = 1;
	tableau[1] = 0x44;

	tableau[2] = raw_position[0]>>24;
	tableau[3] = raw_position[0]>>16;
	tableau[4] = raw_position[0]>>8;
	tableau[5] = raw_position[0]&0x00FF;

	tableau[6] = raw_position[1]>>24;
	tableau[7] = raw_position[1]>>16;
	tableau[8] = raw_position[1]>>8;
	tableau[9] = raw_position[1]&0x00FF;

	trame.message = tableau;

	return trame;
}

// A faire : envoie consigne position brut en pas codeur 2 parametres : [sens]8u ; [pascodeur]16u ;

Trame PiloteGetBuffPosition()
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;

	tableau[0] = 0xC4; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_POSITION;
	nbr_to_send = buff_position_ptr - last_send_ptr;
	last_send_ptr=buff_position_ptr;
	if(nbr_to_send>60) nbr_to_send=60;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*8+4;
	for(i=0;i<nbr_to_send;i++)
	{
		current_send_ptr = buff_position_ptr-nbr_to_send+i;
		tableau[1+2+(i*8)] = buff_position[0][current_send_ptr]>>24;
		tableau[1+3+(i*8)] = buff_position[0][current_send_ptr]>>16;
		tableau[1+4+(i*8)] = buff_position[0][current_send_ptr]>>8;
		tableau[1+5+(i*8)] = buff_position[0][current_send_ptr]&0x00FF;

		tableau[1+6+(i*8)] = buff_position[1][current_send_ptr]>>24;
		tableau[1+7+(i*8)] = buff_position[1][current_send_ptr]>>16;
		tableau[1+8+(i*8)] = buff_position[1][current_send_ptr]>>8;
		tableau[1+9+(i*8)] = buff_position[1][current_send_ptr]&0x00FF;
	}

	trame.message = tableau;

	return trame;
}

int PiloteVitesse(unsigned int id, unsigned int sens, unsigned int vitesse)
{
	double vit = (double)vitesse;

	if(sens == SENS_DROITE)
		vit = -vit;
	
	switch (id)
	{
		case 1: // Moteur 1	
			pwm(MOTEUR_1,vit);	
			break;
		case 2: // Moteur 2
			pwm(MOTEUR_2,vit);
			break;
		case 3: // Moteur 3
			pwm(MOTEUR_3,vit);
			break;
		case 4: // Moteur 4
			pwm(MOTEUR_4,vit);
			break;
	}
//	Motors_SetSpeed(vitesse,MOTEUR_GAUCHE);
//	Motors_SetSpeed(vitesse,MOTEUR_DROIT);
	return 1;
}

int PiloteAcceleration(int acceleration)
{
	Motors_SetAcceleration(acceleration,MOTEUR_GAUCHE);
	Motors_SetAcceleration(acceleration,MOTEUR_DROIT);
	return 1;
}

int PiloteAvancer(double distance)
{
	Avance(distance,0);
	return 1;
}

// Recule de la distance spécifiée
// distance : distance é reculer
int PiloteReculer(double distance)
{
	Avance(-distance,0);
	return 1;
}

// Pivote de l'angle spécifié
// angle : angle de rotation (en degrés)
// direction : coté du pivot (Gauche ou Droite)
int PilotePivoter(double angle, Cote direction)
{
	if(direction==Gauche)	Pivot( angle/100.0,0);
	else					Pivot(-angle/100.0,0);
	return 1;
}

// Effectuer un virage
// angle : angle de rotation (en degrés)
// rayon : rayon du virage
// direction : coté du virage (Gauche ou Droite)
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle)
{
	if(reculer) Virage(direction, rayon, angle/100, 0);
	else	 	Virage(direction, rayon, -angle/100, 0);

	return 1;
}


// Recallage du robot
// s : sens du recallage (Avant ou Arriere)
int PiloteRecallage(Sens s)
{
	Calage(s);

	return 1;
}

// Lance une trajectoire par étapes
// nombreEtapes : nombre de points de passage
// etape : premiere étape du trajet
int PiloteAvancerEtapes(int nombreEtapes, Etape etape)
{
	// TODO : Bah alors, c'est pas encore codé feignasse ?

	return 1;
}

int PiloteOffsetAsserv(int x, int y, int teta)
{
	pos_x = -y;
	pos_y = -x;
	offset_teta = (teta/180*PI) / 100.0 - pos_teta;

	return 1;
}

Trame PiloteEcho(void)
{
	BYTE vbat[6];
	static Trame trameEcho;
	static BYTE msgEcho[2];
/*
	long bat1 = (float)(courant * 0.012693 + 0.343777) * 100;
	long bat2 = (float)(ADC_Results[1] * 0.012693 + 0.343777) * 100;

	vbat[0] = bat1 >> 8; // Etalonnage de compétition !
	vbat[1] = bat1 & 0xFF;
	vbat[2] = bat2 >> 8; // Etalonnage de compétition !
	vbat[3] = bat2 & 0xFF;
*/
	trameEcho.nbChar = 6;
	trameEcho.message = msgEcho;
	msgEcho[0] = 0xC4;
	msgEcho[1] = CMD_REPONSE_ECHO;
	return trameEcho;
}

void EnvoiUART(Trame t)
{
	unsigned char i;
	// Copier trame dans buffer circulaire d'envoi
	for(i=0;i<t.nbChar-3;i++)
	{
		buffer_envoi_uart[ptr_write_buffer_uart++]=t.message[i+3];
		if(ptr_write_buffer_uart >= UART_BUFFER_SIZE)
			ptr_write_buffer_uart=0;
	}
}

void EnvoiUART2(Trame t)
{
	unsigned char i;
	// Copier trame dans buffer circulaire d'envoi
	for(i=0;i<t.nbChar-3;i++)
	{
		buffer_envoi_uart2[ptr_write_buffer_uart2++]=t.message[i+3];
		if(ptr_write_buffer_uart2 >= UART_BUFFER_SIZE2)
			ptr_write_buffer_uart2=0;
	}
}

Trame Retour_Valeurs_Numeriques(void)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[8];
	Etat_Valeurs.nbChar = 8;
	

	Valeurs[0] = UDP_ID;
	Valeurs[1] = CMD_REPONSE_VALEURS_NUMERIQUES;
	Valeurs[2] = PORTA>>8;
	Valeurs[3] = PORTA&0xFF;
	Valeurs[4] = PORTB>>8;
	Valeurs[5] = PORTB&0xFF;
	Valeurs[6] = PORTC>>8;
	Valeurs[7] = PORTC&0xFF;
	
	
	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, param5, param6, param7, param8;

	retour = t;

	// Les messages ne commencant pas par la bone adresse ne nous sont pas adresses
	if(t.message[0] != UDP_ID)
		return t;

	switch(t.message[1])
	{
		case CMD_DEBUG:
			break;

		case CMD_DEMANDE_ECHO:
			//retour = PiloteEcho();
			break;

		case CMD_VITESSE_MOTEUR:
			param1 = t.message[2];						// ID
			param2 = t.message[3];						// SENS
			param3 = t.message[4] * 256 + t.message[5]; // Valeur Vitesse
			PiloteVitesse(param1, param2 , param3);
			break;

		case CMD_ACCELERATION_MOTEUR:
			break;

		case CMD_DEMANDE_VALEURS_ANALOGIQUES:
			return Retour_Valeurs_Analogiques();
			break;

		case TRAME_PILOTAGE_ONOFF:
			switch (t.message[2])
			{
				case ALIMENTATION_CAPTEUR_COULEUR:
					alim_capteur_couleur = t.message[3];
					break;
				case MAKEVACUUM_BACK:
					if(t.message[3])
					{
						PWM2CON1bits.PEN1L = 1;
						MOT1L=0;
						P2DC1 = 4000;
					}
					else
					{
						PWM2CON1bits.PEN1L = 0;
						MOT1L=1;
						P2DC1 = 4000;
					}
					break;
				case MAKEVACUUM_FRONT:
					if(t.message[3])
					{
						PWM2CON1bits.PEN1H = 1;
						MOT1H=0;
						P2DC1 = 4000;
					}
					else
					{
						PWM2CON1bits.PEN1H = 0;
						MOT1H=1;
						P2DC1 = 4000;
					}
				case OPENVACUUM_BACK:
					if(t.message[3])
					{
						PWM1CON1bits.PEN1L = 1;
						MOT4L=0;
						P1DC1 = 4000;
					}
					else
					{
						PWM1CON1bits.PEN1L = 0;
						MOT4L=1;
						P1DC1 = 4000;
					}
					break;
				case OPENVACUUM_FRONT:
					if(t.message[3])
					{
						PWM1CON1bits.PEN1H = 1;
						MOT4H=0;
						P1DC1 = 4000;
					}
					else
					{
						PWM1CON1bits.PEN1H = 0;
						MOT4H=1;
						P1DC1 = 4000;
					}
					break;
			}
			break;

		case CMD_ENVOI_UART:
			EnvoiUART(t);
			break;

		case CMD_ENVOI_UART2:
			EnvoiUART2(t);
			break;

		case CMD_DEMANDE_CAPTEUR_COULEUR:
      		retour = CouleurRGB(t.message[2]);
			break;

		case CMD_STOP_MOTEUR:
			param1 = t.message[2];							// Id moteur
			param2 = t.message[3];							// StopMode
			PiloteStop((unsigned char)param1,(unsigned char)param2);
			break;

		case CMD_MOTEUR_POSITION:
			param1 = t.message[2];							// Id moteur
			param2 = t.message[3] * 256 + t.message[4];		// Position
			MotorPosition(param2,(unsigned char)param1);
			break;

		case CMD_MOTEUR_ORIGIN:
			param1 = t.message[2];							// Id moteur
			MotorPosition(-500,(unsigned char)param1);
			break;

		case CMD_MOTEUR_INIT:
			param1 = t.message[2];							// Id moteur
			switch(param1)
			{
				case 0:
					POS1CNT=0;
					revolutions[0]=0;
					break;
				case 1:
					POS2CNT=0;
					revolutions[1]=0;
					break;
			}
			break;

		case CMD_DEMANDE_VALEURS_NUMERIQUES:
			return Retour_Valeurs_Numeriques();
		
		case CMD_DEMANDE_CAPTEUR_ONOFF:
			return Retour_Capteur_Onoff(t.message[2]);
		default:
			return retour;
	}
	return retour;
}