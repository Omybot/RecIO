#ifndef __ASSER_H__
#define __ASSER_H__

#include "Pilotage.h"

#define N				2						// Nombre de moteurs
#define DEFAULT_KP		12//12						// Coefficient proporionnel 
#define DEFAULT_KI		0.01					// Coefficient integral (inverse !)
#define DEFAULT_KD		120//70						// Coefficient derive
#define CODEUR			4096 					// Increments par tour  : Analog ratiometric 10 % to 90 % of Vsupply 0.33 V ==> 0° ; 2.97 V ==> 360° 
#define VOIE			1						// unused
#define DIAMETRE_ROUE 	46.32					// Diametre de la roue motrice (ou roue codeuse si indépendante) en mm 
#define PI 				3.1415926535897932384	// Ben pi quoi
#define COEFF_ROUE		1.000000000000000		// Coeff d'ajustement pour le diametre de la roue
#define MM_SCALER		DIAMETRE_ROUE*PI/CODEUR // Formule de conversion [pas] ==> [mm]
#define MM_INVSCALER	CODEUR/(DIAMETRE_ROUE*PI) // ==> 1/MM_SCALER marche pas
#define DEFAULT_SPEED	2000//5000						// Vitesse par défaut en mm/s
#define DEFAULT_ACCEL	1500//20000						// Acceleration par défaut en mm/s^2
#define ERROR_ALLOWED	0						// En cas de sifflement moteur intempestif (en pas)


#define LAT_ASCENSEUR_GAUCHE 		LATCbits.LATC7
#define LAT_ASCENSEUR_DROITE 		LATBbits.LATB10
#define LAT_POMPE 		LATBbits.LATB13
#define DIR_ASCENSEUR_GAUCHE 		LATCbits.LATC6
#define DIR_ASCENSEUR_DROITE 		LATBbits.LATB9
#define DIR_POMPE			 		LATBbits.LATB12
#define PWM_ASCENSEUR_GAUCHE		P2DC1
#define PWM_ASCENSEUR_DROITE		P1DC3
#define PWM_BALISE					P1DC1
#define PWM_POMPE					P1DC2


#define DIRG 	LATBbits.LATB12
#define DIRD 	LATBbits.LATB14

#define STOP			0x01
#define VITESSE			0x02
#define ACCELERATION	0x03
#define AVANCE			0x04
#define PIVOT			0x05
#define VIRAGE			0x06
#define DISTANCE		0x08
#define ARRIVE			0x10
#define PIVOTG			0x11
#define PIVOTD			0x12
#define RECULE			0x13
#define COUPURE			0x90
#define COEFF_P			0x20
#define COEFF_I			0x21
#define COEFF_D			0x22
#define ROULEAU_AV		0x81
#define ROULEAU_AR		0x82

#define FALSE			0x00
#define TRUE			0x01


#define AVANT 			0			// Convention 
#define ARRIERE 		1
#define MOTEUR_GAUCHE 	0	// Convetion de merde !		
#define MOTEUR_DROIT 	1
#define GAUCHE 			2			
#define DROITE 			3
#define ON				1
#define OFF				0
#define FREELY			0
#define SMOOTH			1
#define ABRUPT			2

#define MOTION_ACCEL	21
#define MOTION_DECEL	30
#define MOTION_RUN		22
#define MOTION_STOP		0
#define MOTION_OFF		0
#define MOTION_START	1

#define FLAG_ENVOI		0x10
#define FLAG_CALAGE		0x30
#define FLAG_BLOCAGE	0x40


unsigned char Motors_Task(void);
void manual_pid(double moteurga, double moteurdr);
void InitProp(void);
void Avance(double distance, unsigned char wait);
void Go(double distance, unsigned char moteur);
void Pivot(double angle,unsigned char wait);
void Virage(unsigned char cote, double rayon, double angle, unsigned char wait);
double Stop(unsigned char stopmode);


double Motors_GetPosition(unsigned char moteur);
void Motors_GetRawPosition(unsigned int *buffer);

void Motors_DefineHome(unsigned char moteur);
void Motors_Stop(unsigned char stopmode, unsigned char moteur);

unsigned char Motors_IsRunning(unsigned char moteur);
void Motors_SetPosition(double position, unsigned char moteur);
void Motors_SetAcceleration(double acceleration, unsigned char moteur);

void Motors_SetAcceleration_Pivot(double acceleration);
void Motors_SetSpeed_Pivot(double vitesse);
void Motors_SetAcceleration_Ligne(double acceleration);
void Motors_SetSpeed_Ligne(double vitesse);

void Motors_SetSpeed(double vitesse, unsigned char moteur);
void Motors_Power(unsigned char power, unsigned char moteur);
void Motors_Start(unsigned char moteur);

void GotoXY(double x, double y, unsigned char reculer);
unsigned char Motor_Task(void);
void set_pid(double * coeffs);
unsigned char pid(unsigned char power, double * targ_pos,double * real_pos);
char pwm(unsigned char motor, double valeur);
//double _abs(double value);
void Calage(unsigned char reculer);


void Sleepms(unsigned int nbr);
void Sleepus(unsigned int nbr);

#endif // __ASSER_H__
