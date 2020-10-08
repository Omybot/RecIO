#ifndef __ASSER_H__
#define __ASSER_H__

#include "Pilotage.h"


#define N				2						// Nombre de moteurs
#define DEFAULT_KP		50						// Coefficient proporionnel 
#define DEFAULT_KI		0						// Coefficient integral (inverse !)
#define DEFAULT_KD		200						// Coefficient derive
#define CODEUR			1024 					// Nombre de pas par tour moteur (sans le ratio x4)
#define REDUCTEUR		1						// Reducteur utilise en sortie d'arbre moteur (=1 si roue codeuse indépendante)
#define DIAMETRE_ROUE 	35*0.845						// Diametre de la roue motrice (ou roue codeuse si indépendante) en mm 
#define PI 				3.1415926535897932384	// Ben pi quoi
#define VOIE			306.1120747/0.99857145 			// Distance entre les deux roues en mm
#define COEFF_ROUE		1.084082646			// Coeff d'ajustement pour le diametre de la roue
#define MM_SCALER		COEFF_ROUE*DIAMETRE_ROUE*PI/(4*CODEUR*REDUCTEUR) // Formule de conversion [pas]<==>[mm]
#define MM_INVSCALER	4*CODEUR*REDUCTEUR/(COEFF_ROUE*DIAMETRE_ROUE*PI)
#define DEFAULT_SPEED	400						// Vitesse par défaut en mm/s
#define DEFAULT_ACCEL	3000					// Acceleration par défaut en mm/s^2
#define ERROR_ALLOWED	0						// En cas de sifflement moteur intempestif (en pas)


#define MOT1H 	LATCbits.LATC6
#define MOT1L	LATCbits.LATC7
#define MOT2H 	LATBbits.LATB10
#define MOT2L	LATBbits.LATB11
#define MOT3H 	LATBbits.LATB12
#define MOT3L	LATBbits.LATB13
#define MOT4H 	LATBbits.LATB14
#define MOT4L	LATBbits.LATB15

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
#define MOTEUR_1		10
#define MOTEUR_2		20
#define MOTEUR_3		30
#define MOTEUR_4		40
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
#define FLAG_ENVOI0		0x50
#define FLAG_ENVOI1		0x60
#define FLAG_BLOCAGE0	0x70
#define FLAG_BLOCAGE1	0x80


unsigned char Motors_Task(void);
void manual_pid(double moteurga, double moteurdr);
void InitProp(void);
void Avance(double distance, unsigned char wait);
void MotorPosition(double distance, unsigned char id_moteur);
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
void Motors_SetAcceleration_Ligne(double acceleration_debut, double acceleration_fin);
void Motors_SetSpeed_Ligne(double vitesse);

void Motors_SetSpeed(double vitesse, unsigned char moteur);
void Motors_Power(unsigned char power);
void Motors_Start(unsigned char moteur);


void GotoXY(double x, double y, unsigned char reculer);
unsigned char Motor_Task(void);
void set_pid(double * coeffs);
unsigned char pid(unsigned char power, double * targ_pos,double * real_pos);
char pwm(unsigned char motor, double valeur);
//double _abs(double value);
void Calage(unsigned char reculer);
void Deplacement_Polaire(void);


void Sleepms(unsigned int nbr);
void Sleepus(unsigned int nbr);


#endif // __ASSER_H__
