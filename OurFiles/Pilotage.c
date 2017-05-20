#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>
#include "CDS5516.h"
#include "init.h"
#include "Stepper.h"

#define UART_BUFFER_SIZE 100

//Define Capteur Couleur
#define LED LATAbits.LATA8
#define S2 LATAbits.LATA10
#define S3 LATAbits.LATA7
//	Extern Capteur Couleur
extern unsigned int Tab_Capteur_Couleur[8];

// UART
extern 	unsigned char flag_envoi_uart,buffer_envoi_uart[UART_BUFFER_SIZE],ptr_write_buffer_uart;

// ATTENTION /!\ Ces fonctions ne doivent pas �tre bloquantes
extern unsigned char detection_pompe;
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
double BAUDRATE, BRGVAL, FCY = 40000000;
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

//Variable extern pour pince fruitmouth
unsigned int Periode_pince_droite_haute = 0;//INIT_PINCE;
unsigned int Periode_pince_gauche_haute = 0;//INIT_PINCE;
unsigned int Periode_pince_droite_basse = 0;//INIT_PINCE;
unsigned int Periode_pince_gauche_basse = 0;//INIT_PINCE;

//Variable Capteur Couleur
unsigned int Valeur_Capteur_Couleur = 24;
unsigned int Cpt_Tmr_Periode = 0;

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

// DEBUG
Trame PiloteDebug0(Trame t)
{
	t = Couleur_Balle();
	return t;
}

Trame PiloteDebug1(Trame t)
{
	return t;
}

Trame PiloteDebug2(Trame t)
{
	static int nbChar = 5;
	BYTE timePeriod[nbChar];
	BYTE colorCtrl = t.message[3];

	S2 = (colorCtrl & 0x04)>>2;
	S3 = (colorCtrl & 0x02)>>1;
	LED = colorCtrl & 0x01;
	timePeriod[0] = 0xC4;
	timePeriod[1] = CMD_DEBUG;
	timePeriod[2] = 0x02;
	timePeriod[3] = Valeur_Capteur_Couleur>>8;
	timePeriod[4] = Valeur_Capteur_Couleur&0x00FF;

	t.nbChar  = nbChar;
	t.message = timePeriod;

	return t;
}

Trame PiloteDebug3(Trame t)
{

	return t;
}

Trame PiloteDebug4(Trame t)
{
	return t;
}

Trame PiloteDebug5(Trame t)
{
	return t;
}

Trame PiloteDebug6(Trame t)
{
	return t;
}

Trame PiloteDebug7(Trame t)
{
	return t;
}

Trame PiloteDebug8(Trame t)
{
	return t;
}

Trame PiloteDebug9(Trame t)
{
	return t;
}



void Init_Servos(void)
{
	// TODO
}

//Initalisation Alimentation
void Init_Alimentation(void)
{
}

//Fonction qui renvoie sous forme de trame(UDP) la couleur de l'equipe
Trame Couleur_Equipe(void)
{
	Trame Etat_Couleur_Equipe;
	static BYTE Couleur[3];
	Etat_Couleur_Equipe.nbChar = 3;

	Couleur[0] = 0xC4;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = MATCH_COULEUR;

	Etat_Couleur_Equipe.message = Couleur;

	return Etat_Couleur_Equipe;
}

Trame Presence_Bouchon(void)
{
	Trame Etat_Bouchon;
	static BYTE Bouchon[4];
	Etat_Bouchon.nbChar = 4;

	Bouchon[0] = 0xC4;

	Etat_Bouchon.message = Bouchon;

	return Etat_Bouchon;
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


Trame Presence_Feu(unsigned char id_feu)
{
	Trame Etat_Feu;
	static BYTE Feu[4];
	Etat_Feu.nbChar = 4;

	Feu[0] = 0xC4;
	Feu[1] = CMD_REPONSE_CAPTEUR_ONOFF;


	Etat_Feu.message = Feu;

	return Etat_Feu;
}


Trame Presence_Jack(void)
{
	Trame Etat_Jack;
	static BYTE Jack[3];
	Etat_Jack.nbChar = 3;

	Jack[0] = 0xC4;
	Jack[1] = CMD_REPONSE_PRESENCE_JACK;
	Jack[2] = !MATCH_JACK;

	Etat_Jack.message = Jack;

	return Etat_Jack;
}

//Set baud rate microcontroleur
void PiloteUARTSetBaudrateMicro(unsigned char newBaud)
{
	switch(newBaud)
	{
		case 207: //9600 Baud
			UBRVALUE = UXBRG_AX12_9600;
			CDS5516SetUXBRG(UXBRG_AX12_9600);
			break;
		case 103: //19200 Baud
			UBRVALUE = UXBRG_AX12_19200;
			CDS5516SetUXBRG(UXBRG_AX12_19200);
			break;
		case 34: //57600 Baud
			UBRVALUE = UXBRG_AX12_57600;
			CDS5516SetUXBRG(UXBRG_AX12_57600);
			break;
		case 16: //115200 Baud
			UBRVALUE = UXBRG_AX12_115200;
			CDS5516SetUXBRG(UXBRG_AX12_115200);
			break;
		case 9: //200000 Baud
			UBRVALUE = UXBRG_AX12_200000;
			CDS5516SetUXBRG(UXBRG_AX12_200000);
			break;
		case 7: //250000 Baud
			UBRVALUE = UXBRG_AX12_250000;
			CDS5516SetUXBRG(UXBRG_AX12_250000);
			break;
		case 4: //400000 Baud
			UBRVALUE = UXBRG_AX12_400000;
			CDS5516SetUXBRG(UXBRG_AX12_400000);
			break;
		case 3: //500000 Baud
			UBRVALUE = UXBRG_AX12_500000;
			CDS5516SetUXBRG(UXBRG_AX12_500000);
			break;
		case 1: //1000000 Baud
			UBRVALUE = UXBRG_AX12_1000000;
			CDS5516SetUXBRG(UXBRG_AX12_1000000);
			break;
	}
}

Trame PiloteGotoXY(int x,int y, unsigned char x_negatif, unsigned char y_negatif)
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;

	GotoXY((double)x,(double)y,0);

	tableau[0] = 1;
	tableau[1] = 0x13;
	tableau[2] = (int)xyangle>>8;
	tableau[3] = (int)xyangle&0x00FF;
	tableau[4] = (int)xydistance>>8;
	tableau[5] = (int)xydistance&0x00FF;

	trame.message = tableau;

	return trame;

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

void PiloteAlimentation(char onOff)
{
}

int PiloteVitesse(int vitesse)
{
	Motors_SetSpeed(vitesse,MOTEUR_GAUCHE);
	Motors_SetSpeed(vitesse,MOTEUR_DROIT);
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

// Stoppe le robot
// mode : mode de stop (Abrupt, Smooth, Freely)
int PiloteStop(unsigned char stopmode)
{
	int distanceRestante;
	Trame envoiReste;
	static BYTE messReste[2];
	messReste[0] = 0xC4;
	messReste[1] = 0x60;
	envoiReste.nbChar = 4;

	distanceRestante = (int)Stop(stopmode);

	messReste[2] = distanceRestante >> 8;
	messReste[3] = distanceRestante & 0xFF;

	//envoiReste.message = messReste;

	while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));

	EnvoiUserUdp(envoiReste);

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

Trame PiloteServoDemandeConfigAlarmeLED(char id)
{
	int config = CDS5516DemandeConfigAlarmeLED(103, id);
	trameServo.nbChar = 11;
	msgServo[2] = CMD_SERVO_RETOUR_CFG_ALARME_LED;
	msgServo[3] = id;
	if(config & 0b0000001)
		msgServo[4] = 1;
	else
		msgServo[4] = 0;
	if(config & 0b0000010)
		msgServo[5] = 1;
	else
		msgServo[5] = 0;
	if(config & 0b0000100)
		msgServo[6] = 1;
	else
		msgServo[6] = 0;
	if(config & 0b0001000)
		msgServo[7] = 1;
	else
		msgServo[7] = 0;
	if(config & 0b0010000)
		msgServo[8] = 1;
	else
		msgServo[8] = 0;
	if(config & 0b0100000)
		msgServo[9] = 1;
	else
		msgServo[9] = 0;
	if(config & 0b1000000)
		msgServo[10] = 1;
	else
		msgServo[10] = 0;

	return trameServo;
}

//Stepper
void PiloteRecalageStepper(void){

	InitPosStepper();
}

Trame PiloteServoDemandeConfigAlarmeShutdown(char id)
{
	int config = CDS5516DemandeConfigAlarmeShutdown(103, id);
	trameServo.nbChar = 11;
	msgServo[2] = CMD_SERVO_RETOUR_CFG_ALARME_SHUTDOWN;
	msgServo[3] = id;

	if(config & 0b0000001)
		msgServo[4] = 1;
	else
		msgServo[4] = 0;
	if(config & 0b0000010)
		msgServo[5] = 1;
	else
		msgServo[5] = 0;
	if(config & 0b0000100)
		msgServo[6] = 1;
	else
		msgServo[6] = 0;
	if(config & 0b0001000)
		msgServo[7] = 1;
	else
		msgServo[7] = 0;
	if(config & 0b0010000)
		msgServo[8] = 1;
	else
		msgServo[8] = 0;
	if(config & 0b0100000)
		msgServo[9] = 1;
	else
		msgServo[9] = 0;
	if(config & 0b1000000)
		msgServo[10] = 1;
	else
		msgServo[10] = 0;

	return trameServo;
}

Trame PiloteServoDemandeConfigEcho(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_CFG_ECHO;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeConfigEcho(103, id);
	return trameServo;
}

Trame PiloteServoDemandeCompliance(char id)
{
	trameServo.nbChar = 8;
	msgServo[2] = CMD_SERVO_RETOUR_COMPLIANCE_PARAMS;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeCCWSlope(103, id);
	delay10ms();
	msgServo[5] = CDS5516DemandeCCWMargin(103, id);
	delay10ms();
	msgServo[6] = CDS5516DemandeCWSlope(103, id);
	delay10ms();
	msgServo[7] = CDS5516DemandeCWMargin(103, id);
	return trameServo;
}

Trame PiloteServoDemandeCoupleActive(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_COUPLE_ACTIVE;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeCoupleActive(103, id);
	return trameServo;
}

Trame PiloteServoDemandeCoupleMaximum(char id)
{
	int valeur = CDS5516DemandeCoupleMaximum(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_COUPLE_MAX;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeCoupleCourant(char id)
{
	int valeur = CDS5516DemandeCoupleCourant(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_COUPLE_COURANT;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoPing(char id)
{
	int valeur=CDS5516Ping(103, id);
	trameServo.nbChar = 11;
	msgServo[2] = CMD_SERVO_RETOUR_ERREURS;
	msgServo[3] = id;
	msgServo[4] = valeur;
	msgServo[5] = valeur;
	msgServo[6] = valeur;
	msgServo[7] = valeur;
	msgServo[8] = valeur;
	msgServo[9] = valeur;
	msgServo[10] = valeur;

	msgServo[4] = (msgServo[4]>>1)&0x01;
	msgServo[5] = (msgServo[5]>>4)&0x01;
	msgServo[6] = (msgServo[6])&0x01;
	msgServo[7] = (msgServo[7]>>6)&0x01;
	msgServo[8] = (msgServo[8]>>2)&0x01;
	msgServo[9] = (msgServo[9]>>5)&0x01;
	msgServo[10] = (msgServo[10]>>3)&0x01;

	return trameServo;
}

Trame PiloteServoDemandeStatusLevel(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_STATUS_LEVEL;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeStatusReturnLevel(103, id);
	return trameServo;
}

Trame PiloteServoDemandeTensionMax(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TENSION_MAX;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTensionMax(103, id);
	return trameServo;
}

Trame PiloteServoDemandeTensionMin(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TENSION_MIN;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTensionMin(103, id);
	return trameServo;
}

Trame PiloteServoDemandeTemperatureMax(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TEMPERATURE_MAX;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTemperatureMax(103, id);
	return trameServo;
}

Trame PiloteServoDemandeLed(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_LED;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeLed(103, id);
	return trameServo;
}

Trame PiloteServoDemandeMouvement(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_MOUVEMENT;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeMouvement(103, id);
	return trameServo;
}

Trame PiloteServoDemandeModele(char id)
{
	int valeur = CDS5516DemandeModele(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_NUMERO_MODELE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionActuelle(char id)
{
	int valeur = CDS5516DemandePositionActuelle(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_ACTUELLE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionCible(char id)
{
	int valeur = CDS5516DemandePositionCible(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_CIBLE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionMax(char id)
{
	int valeur = CDS5516DemandePositionMax(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_MAX;
	msgServo[3] = id;
	msgServo[4] = (int)valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandePositionMin(char id)
{
	int valeur = CDS5516DemandePositionMin(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_POSITION_MIN;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeTemperature(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TEMPERATURE;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTemperature(103, id);
	return trameServo;
}

Trame PiloteServoDemandeTension(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_TENSION;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeTension(103, id);
	return trameServo;
}

Trame PiloteServoDemandeVersionFirmware(char id)
{
	trameServo.nbChar = 5;
	msgServo[2] = CMD_SERVO_RETOUR_VERSION_FIRMWARE;
	msgServo[3] = id;
	msgServo[4] = CDS5516DemandeVersionFirmware(103, id);
	return trameServo;
}

Trame PiloteServoDemandeVitesseActuelle(char id)
{
	int valeur = CDS5516DemandeVitesseActuelle(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_VITESSE_ACTUELLE;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeVitesseMax(char id)
{
	int valeur = CDS5516DemandeVitesseMax(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_VITESSE_MAX;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeCoupleLimitMax(char id)
{
	int valeur = CDS5516DemandeCoupleLimitMax(103, id);
	trameServo.nbChar = 6;
	msgServo[2] = CMD_SERVO_RETOUR_COUPLE_LIMIT_MAX;
	msgServo[3] = id;
	msgServo[4] = valeur >> 8;
	msgServo[5] = valeur & 0xFF;
	return trameServo;
}

Trame PiloteServoDemandeAllIn(char id)
{
	char* tabResultat;
	char iTab;

	tabResultat = CDS5516DemandeMessageAllIn(id,0x00);
	trameServo.nbChar = NB_CHAR_ALL_IN + 4;
	msgServo[2] = CMD_SERVO_RETOUR_ALL_IN;
	msgServo[3] = id;

	for(iTab = 0; iTab < NB_CHAR_ALL_IN; iTab++)
		msgServo[4+iTab] = tabResultat[3+iTab];

	return trameServo;
}

int PiloteServoEnvoiAlarmeLED(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	CDS5516EnvoiAlarmeLED(103 , id, inputVoltage, angleLimit, overheating, range, checksum, overload, instruction);
	return 1;
}

int PiloteServoEnvoiAlarmeShutdown(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	CDS5516EnvoiAlarmeShutdown(103, id, inputVoltage, angleLimit, overheating, range, checksum, overload, instruction);
	return 1;
}

int PiloteServoEnvoiComplianceParams(char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin)
{
	CDS5516EnvoiComplianceParams(103, id, CCWSlope, CCWMargin, CWSlope, CWMargin);
	return 1;
}

int PiloteServoEnvoiCoupleActive(char id, char coupleActive)
{
	CDS5516EnvoiCoupleActive(103, id, coupleActive);
	return 1;
}

int PiloteServoEnvoiCoupleMaximum(char id, unsigned int coupleMax)
{
	CDS5516EnvoiCoupleMaximum(103, id, coupleMax);
	return 1;
}

int PiloteServoEnvoiCoupleLimitMax(char id, unsigned int coupleLimitMax)
{
	CDS5516EnvoiCoupleLimitMax(103, id, coupleLimitMax);
	return 1;
}

int PiloteServoEnvoiTensionMax(char id, unsigned int TensionMax)
{
	CDS5516EnvoiTensionMax(103, id, TensionMax);
	return 1;
}

int PiloteServoEnvoiTensionMin(char id, unsigned int TensionMin)
{
	CDS5516EnvoiTensionMin(103, id, TensionMin);
	return 1;
}

int PiloteServoEnvoiTemperatureMax(char id, unsigned int TemperatureMax)
{
	CDS5516EnvoiTemperatureMax(103, id, TemperatureMax);
	return 1;
}

int PiloteServoEnvoiId(char id, char nouvelId)
{
	CDS5516EnvoiId(103, id, nouvelId);
	return 1;
}

int PiloteServoEnvoiLed(char id, char ledAllume)
{
	CDS5516EnvoiLed(103, id, ledAllume);
	return 1;
}

int PiloteServoEnvoiPosistionCible(char id, unsigned int positionCible)
{
	CDS5516EnvoiPosistionCible(103, id, positionCible);
	return 1;
}

int PiloteServoEnvoiPosistionMax(char id, unsigned int positionMax)
{
	CDS5516EnvoiPositionMax(103, id, positionMax);
	return 1;
}

int PiloteServoEnvoiPosistionMin(char id, unsigned int positionMin)
{
	CDS5516EnvoiPositionMin(103, id, positionMin);
	return 1;
}

int PiloteServoEnvoiVitesseMax(char id, unsigned int vitesseMax)
{
	CDS5516EnvoiVitesseMax(103, id, vitesseMax);
	return 1;
}

int PiloteServoReset(char id)
{
	CDS5516Reset(103, id);
	return 1;
}

int PiloteServoEnvoiBaudrate(char id, unsigned char newBaud)
{
	CDS5516EnvoiBaudrate(103,id, newBaud);
	return 1;
}



Trame PiloteEcho(void)
{
	BYTE vbat[6];
	static Trame trameEcho;
	static BYTE msgEcho[6];

	long bat1 = (float)(courant * 0.012693 + 0.343777) * 100;
	long bat2 = (float)(ADC_Results[1] * 0.012693 + 0.343777) * 100;

	vbat[0] = bat1 >> 8; // Etalonnage de compétition !
	vbat[1] = bat1 & 0xFF;
	vbat[2] = bat2 >> 8; // Etalonnage de compétition !
	vbat[3] = bat2 & 0xFF;

	trameEcho.nbChar = 6;
	trameEcho.message = msgEcho;
	msgEcho[0] = 0xC4;
	msgEcho[1] = CMD_REPONSE_ECHO;
	msgEcho[2] = (BYTE) vbat[0];
	msgEcho[3] = (BYTE) vbat[1];
	msgEcho[4] = (BYTE) vbat[2];
	msgEcho[5] = (BYTE) vbat[3];
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

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, param5, param6, param7, param8;

	retour = t;

	// Les messages ne commencant pas par 0xC4 ne nous sont pas adressés (RecMove)
	if(t.message[0] != 0xC4)
		return t;

	switch(t.message[1])
	{
		case CMD_DEBUG:
			param1 = t.message[2];							// Numero
			switch(param1)
			{
				case 0:
					retour = PiloteDebug0(t);
					break;
				case 1:
					retour = PiloteDebug1(t);
					break;
				case 2:
					retour = PiloteDebug2(t);
					break;
				case 3:
					retour = PiloteDebug3(t);
					break;
				case 4:
					retour = PiloteDebug4(t);
					break;
				case 5:
					retour = PiloteDebug5(t);
					break;
				case 6:
					retour = PiloteDebug6(t);
					break;
				case 7:
					retour = PiloteDebug7(t);
					break;
				case 8:
					retour = PiloteDebug8(t);
					break;
				case 9:
					retour = PiloteDebug9(t);
					break;
			}
		break;

		case CMD_DEMANDE_ECHO:
			//retour = PiloteEcho();
			break;

		case CMD_VITESSE_MOTEUR:
			param1 = t.message[3] * 256 + t.message[4];
			switch (t.message[2])
			{
				case 1: // Moteur 1
					pwm(MOTEUR_1,(double)((int)param1));			
					break;
				case 2: // Moteur 2
					pwm(MOTEUR_2,(double)((int)param1));			
					break;
				case 3: // Moteur 3
					pwm(MOTEUR_3,(double)((int)param1));			
					break;
				case 4: // Moteur 4
					pwm(MOTEUR_4,(double)((int)param1));			
					break;
			}
			break;

		case CMD_ACCELERATION_MOTEUR:
			break;

		case CMD_MOTEUR_POSITION:
			break;


		case CMD_ENVOI_BAUDRATE_MICRO:
			param1 = t.message[2];
			PiloteUARTSetBaudrateMicro(param1);
			break;

		case CMD_DEMANDE_VALEURS_ANALOGIQUES:
			return Retour_Valeurs_Analogiques();
			break;

		case CMD_ACTIONNEUR_ONOFF:
			switch (t.message[2])
			{
			}
			break;

		case CMD_SERVOMOTEUR:

			msgServo[0] = 0xC4;
			msgServo[1] = CMD_SERVOMOTEUR;
			trameServo.message = msgServo;

			switch(t.message[2])
			{
				case CMD_SERVO_DEMANDE_ALL_IN:
					param1 = t.message[3];
					return PiloteServoDemandeAllIn(param1);
					break;
				case CMD_SERVO_DEMANDE_CFG_ALARME_LED:
					param1 = t.message[3];
					return PiloteServoDemandeConfigAlarmeLED(param1);
					break;
				case CMD_SERVO_DEMANDE_CFG_ALARME_SHUTDOWN:
					param1 = t.message[3];
					return PiloteServoDemandeConfigAlarmeShutdown(param1);
					break;
				case CMD_SERVO_DEMANDE_CFG_ECHO:
					param1 = t.message[3];
					return PiloteServoDemandeConfigEcho(param1);
					break;
				case CMD_SERVO_DEMANDE_COMPLIANCE_PARAMS:
					param1 = t.message[3];
					return PiloteServoDemandeCompliance(param1);
					break;
				case CMD_SERVO_DEMANDE_COUPLE_ACTIVE:
					param1 = t.message[3];
					return PiloteServoDemandeCoupleActive(param1);
					break;
				case CMD_SERVO_DEMANDE_COUPLE_MAX:
					param1 = t.message[3];
					return PiloteServoDemandeCoupleMaximum(param1);
					break;
				case CMD_SERVO_DEMANDE_COUPLE_LIMIT_MAX:
					param1 = t.message[3];
					return PiloteServoDemandeCoupleLimitMax(param1);
					break;
				case CMD_SERVO_DEMANDE_COUPLE_COURANT:
					param1 = t.message[3];
					return PiloteServoDemandeCoupleCourant(param1);
					break;
				case CMD_SERVO_DEMANDE_ERREURS:
					param1 = t.message[3];
					return PiloteServoPing(param1);
					break;
				case CMD_SERVO_DEMANDE_STATUS_LEVEL:
					param1 = t.message[3];
					return PiloteServoDemandeStatusLevel(param1);
					break;
				case CMD_SERVO_DEMANDE_LED:
					param1 = t.message[3];
					return PiloteServoDemandeLed(param1);
					break;
				case CMD_SERVO_DEMANDE_MOUVEMENT:
					param1 = t.message[3];
					return PiloteServoDemandeMouvement(param1);
					break;
				case CMD_SERVO_DEMANDE_NUMERO_MODELE:
					param1 = t.message[3];
					return PiloteServoDemandeModele(param1);
					break;
				case CMD_SERVO_DEMANDE_POSITION_ACTUELLE:
					param1 = t.message[3];
					return PiloteServoDemandePositionActuelle(param1);
					break;
				case CMD_SERVO_DEMANDE_POSITION_CIBLE:
					param1 = t.message[3];
					return PiloteServoDemandePositionCible(param1);
					break;
				case CMD_SERVO_DEMANDE_POSITION_MAX:
					param1 = t.message[3];
					return PiloteServoDemandePositionMax(param1);
					break;
				case CMD_SERVO_DEMANDE_POSITION_MIN:
					param1 = t.message[3];
					return PiloteServoDemandePositionMin(param1);
					break;
				case CMD_SERVO_DEMANDE_TEMPERATURE:
					param1 = t.message[3];
					return PiloteServoDemandeTemperature(param1);
					break;
				case CMD_SERVO_DEMANDE_TEMPERATURE_MAX:
					param1 = t.message[3];
					return PiloteServoDemandeTemperatureMax(param1);
					break;
				case CMD_SERVO_DEMANDE_TENSION:
					param1 = t.message[3];
					return PiloteServoDemandeTension(param1);
					break;
				case CMD_SERVO_DEMANDE_TENSION_MAX:
					param1 = t.message[3];
					return PiloteServoDemandeTensionMax(param1);
					break;
				case CMD_SERVO_DEMANDE_TENSION_MIN:
					param1 = t.message[3];
					return PiloteServoDemandeTensionMin(param1);
					break;
				case CMD_SERVO_DEMANDE_VERSION_FIRMWARE:
					param1 = t.message[3];
					return PiloteServoDemandeVersionFirmware(param1);
					break;
				case CMD_SERVO_DEMANDE_VITESSE_ACTUELLE:
					param1 = t.message[3];
					return PiloteServoDemandeVitesseActuelle(param1);
					break;
				case CMD_SERVO_DEMANDE_VITESSE_MAX:
					param1 = t.message[3];
					return PiloteServoDemandeVitesseMax(param1);
					break;
				case CMD_SERVO_ENVOI_BAUDRATE:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiBaudrate(param1, param2);
					break;
				case CMD_SERVO_ENVOI_CFG_ALARME_LED:
					param1 = t.message[3];
					param2 = t.message[4];
					param3 = t.message[5];
					param4 = t.message[6];
					param5 = t.message[7];
					param6 = t.message[8];
					param7 = t.message[9];
					param8 = t.message[10];
					PiloteServoEnvoiAlarmeLED(param1, param2, param3, param4, param5, param6, param7, param8);
					break;
				case CMD_SERVO_ENVOI_CFG_ALARME_SHUTDOWN:
					param1 = t.message[3];
					param2 = t.message[4];
					param3 = t.message[5];
					param4 = t.message[6];
					param5 = t.message[7];
					param6 = t.message[8];
					param7 = t.message[9];
					param8 = t.message[10];
					PiloteServoEnvoiAlarmeShutdown(param1, param2, param3, param4, param5, param6, param7, param8);
					break;
				case CMD_SERVO_ENVOI_CFG_ECHO:
					// TODO
					break;
				case CMD_SERVO_ENVOI_COMPLIANCE_PARAMS:
					param1 = t.message[3];
					param2 = t.message[4];
					param3 = t.message[5];
					param4 = t.message[6];
					param5 = t.message[7];
					PiloteServoEnvoiComplianceParams(param1, param2, param3, param4, param5);
					break;
				case CMD_SERVO_ENVOI_COUPLE_ACTIVE:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiCoupleActive(param1, param2);
					break;
				case CMD_SERVO_ENVOI_COUPLE_MAX:
					param1 = t.message[3];
					param2 = t.message[4] * 256 + t.message[5];
					PiloteServoEnvoiCoupleMaximum(param1, param2);
					break;
				case CMD_SERVO_ENVOI_COUPLE_LIMIT_MAX:
					param1 = t.message[3];
					param2 = t.message[4] * 256 + t.message[5];
					PiloteServoEnvoiCoupleLimitMax(param1, param2);
					break;
				case CMD_SERVO_ENVOI_TENSION_MAX:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiTensionMax(param1, param2);
					break;
				case CMD_SERVO_ENVOI_TENSION_MIN:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiTensionMin(param1, param2);
					break;
				case CMD_SERVO_ENVOI_TEMPERATURE_MAX:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiTemperatureMax(param1, param2);
					break;
				case CMD_SERVO_ENVOI_ID:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiId(param1, param2);
					break;
				case CMD_SERVO_ENVOI_LED:
					param1 = t.message[3];
					param2 = t.message[4];
					PiloteServoEnvoiLed(param1, param2);
					break;
				case CMD_SERVO_ENVOI_POSITION_CIBLE:
					param1 = t.message[3];
					param2 = t.message[4] * 256 + t.message[5];
					PiloteServoEnvoiPosistionCible(param1, param2);
					break;
				case CMD_SERVO_ENVOI_POSITION_MAX:
					param1 = t.message[3];
					param2 = t.message[4] * 256 + t.message[5];
					PiloteServoEnvoiPosistionMax(param1, param2);
					break;
				case CMD_SERVO_ENVOI_POSITION_MIN:
					param1 = t.message[3];
					param2 = t.message[4] * 256 + t.message[5];
					PiloteServoEnvoiPosistionMin(param1, param2);
					break;
				case CMD_SERVO_ENVOI_VITESSE_MAX:
					param1 = t.message[3];
					param2 = t.message[4] * 256 + t.message[5];
					PiloteServoEnvoiVitesseMax(param1, param2);
					break;
				case CMD_SERVO_RESET:
					param1 = t.message[3];
					PiloteServoReset(param1);
					break;
			}
		case CMD_ENVOI_UART:
			EnvoiUART(t);
			break;

		case CMD_DEMANDE_CAPTEUR_COULEUR:
      		retour = CouleurRGB(t.message[2]);
			break;
	}
	return retour;
}
