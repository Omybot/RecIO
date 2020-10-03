#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "Types.h"
#include "UserUdp.h"


int PiloteStop(unsigned char stopmode,unsigned char id_moteur);


static Trame trameServo;
static BYTE msgServo[60];

Trame Retour_Capteur_Onoff(unsigned char id_capteur);

Trame PiloteGotoXY(int x, int y, unsigned char x_negatif, unsigned char y_negatif);

Trame StatusMonitor(void);

void Init_Servos(void);
Trame PilotePIDRessource();
void PilotePIDInit(void);
Trame PiloteGetPosition(unsigned char cote);
Trame PiloteGetRawPosition(void);
Trame PiloteGetLongPosition(void);
Trame PiloteGetBuffPosition(void);
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd);
void PilotePIDManual(unsigned int gauche,unsigned int droite);
void PilotePIDBridage(unsigned int value);
Trame PilotePIDErreurs(void);
void PilotePIDFeedforward(unsigned int value);
void Assiette_Position(unsigned int vitesse);

Trame PilotePositionXYT(void);

Trame Couleur_Equipe(void);
Trame Retour_Valeurs_Analogiques(void);
Trame Presence_Jack(void);

int Coupure(void);

//Uart
void PiloteUARTSetBaudrateMicro(unsigned char newBaud);
Trame PiloteEcho(void);

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

//Initialisation timer 2 et 4
void Init_Timer (void);

//Initialisation Alimentation
void Init_Alimentation(void);

// Capteur de couleur
typedef struct Rgb
{
  unsigned char red;
  unsigned char green;
  unsigned char blue;
}Rgb;
unsigned int Send_Variable_Capteur_Couleur(void);
Trame Couleur_Balle(void);
Trame CouleurRGB(int Id);
double period2frequency(unsigned int period);
Rgb frequency2RGB(double freqClear, double freqRed, double freqGreen, double freqBlue);

//Delay
void delay(void);
void delay10ms(void);
void delays(void);
void delayms(void);

//ASSERVISSEMENT

int PiloteVitesse(unsigned int id, unsigned int sens, unsigned int vitesse);
int PiloteAcceleration(int acceleration);
int PiloteAvancer(double distance);
int PiloteReculer(double distance);
int PilotePivoter(double angle, Cote direction);
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle);
int PiloteStop(unsigned char id_moteur, unsigned char stopmode);
int PiloteRecallage(Sens s);
int PiloteAvancerEtapes(int nombreEtapes, Etape etape);
int PiloteValiderEtapes(int numEtape);
int PiloteOffsetAsserv(int x, int y, int teta);

Trame Retour_Valeurs_Numeriques(void);

//Analyse Trame
Trame AnalyseTrame(Trame t);

//Stepper
void PiloteRecalageStepper(void);

// Capteur de couleur
#define CMD_DEMANDE_CAPTEUR_COULEUR 	0x52
#define CMD_REPONSE_CAPTEUR_COULEUR 	0x53

// Define Servos
#define CPT_PERIODE_20MS 6250
#define RISING_EDGE 1
#define FALLING_EDGE 0

// Moteur sens
#define SENS_DROITE 3
#define SENS_GAUCHE 2

// Deplacements
#define	CMD_AVANCER						0x01
#define	CMD_PIVOTER						0x03
#define	CMD_VIRAGE						0x04
#define	CMD_STOP						0x05
#define	CMD_GOTOXY						0x06
#define CMD_FINDEPLACEMENT				0x12
#define CMD_BLOCAGE						0x13

// Asservissement
#define CMD_DEMANDEPOSITION				0x30
#define CMD_RETOURPOSITION				0x31
#define	CMD_VITESSE_LIGNE				0x32
#define	CMD_ACCELERATION_LIGNE			0x33
#define	CMD_VITESSE_PIVOT				0x34
#define	CMD_ACCELERATION_PIVOT			0x35
#define CMD_ENVOI_PID 					0x36
#define CMD_OFFSETASSERV				0x37

// Debug asservissement
#define CMD_DEMANDE_BUFF_POSITION		0x43
#define CMD_REPONSE_BUFF_POSITION		0x44
#define CMD_CONSIGNE_POSITION			0x45
#define CMD_DEMANDE_BUFF_STATUS			0x46
#define CMD_REPONSE_BUFF_STATUS			0x47

// Actionneurs
#define CMD_MOTEUR_ORIGIN				0x63
#define CMD_MOTEUR_INIT					0x64
#define TRAME_PILOTAGE_ONOFF 			0x65
#define CMD_MOTEUR_POSITION				0x66
#define CMD_VITESSE_MOTEUR				0x67
#define CMD_ACCELERATION_MOTEUR			0x68
#define CMD_STOP_MOTEUR					0x69
#define CMD_REPONSE_FIN					0x70
#define CMD_REPONSE_BLOCAGE				0x71

// ID Actionneurs
#define ID_MOTEUR_ASCENSEUR_GAUCHE		0x00
#define ID_MOTEUR_ASCENSEUR_DROITE		0x01
#define ID_MOTEUR_POMPE_GAUCHE			0x02
#define ID_MOTEUR_POMPE_DROITE			0x03

// Capteurs onOff
#define CMD_DEMANDE_CAPTEUR_ONOFF		0x74
#define CMD_REPONSE_CAPTEUR_ONOFF		0x75

// Passerelle UART
#define CMD_ENVOI_UART					0xA0
#define CMD_RECEPTION_UART				0xA1
#define CMD_ENVOI_UART2					0xA4
#define CMD_RECEPTION_UART2				0xA5

// Capteurs
#define CMD_DEMANDE_VALEURS_ANALOGIQUES	0x76
#define CMD_REPONSE_VALEURS_ANALOGIQUES	0x77
#define CMD_DEMANDE_VALEURS_NUMERIQUES	0x78
#define CMD_REPONSE_VALEURS_NUMERIQUES	0x79

#define CMD_DETECTION_SECU_ASCENSEUR	0x82

// Alimentation
#define CMD_ALIMENTATION				0x80

// Diagnostic
#define	CMD_DEBUG						0xEE
#define	CMD_DEMANDE_ECHO				0xF0
#define CMD_REPONSE_ECHO				0xF0


// Liste des actionneurs
#define ALIMENTATION_CAPTEUR_COULEUR	0x01
#define MAKEVACUUM_BACK					0x13
#define MAKEVACUUM_FRONT				0x11
#define OPENVACUUM_BACK					0x22
#define OPENVACUUM_FRONT				0x20

#define VACUOSTAT_BACK					0x13
#define VACUOSTAT_FRONT					0x11


#endif // __PILOTAGE_H__
