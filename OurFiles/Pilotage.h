#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "Types.h"
#include "UserUdp.h"


int PiloteStop(unsigned char stopmode,unsigned char id_moteur);


static Trame trameServo;
static BYTE msgServo[60];

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

// SERVOMOTEURS

Trame PiloteServoDemandeConfigAlarmeLED(char id);
Trame PiloteServoDemandeConfigAlarmeShutdown(char id);
Trame PiloteServoDemandeConfigEcho(char id);
Trame PiloteServoDemandeCompliance(char id);
Trame PiloteServoDemandeCoupleActive(char id);
Trame PiloteServoDemandeCoupleMaximum(char id);
Trame PiloteServoDemandeCoupleCourant(char id);
Trame PiloteServoPing(char id);
Trame PiloteServoDemandeStatusLevel(char id);
Trame PiloteServoDemandeLed(char id);
Trame PiloteServoDemandeMouvement(char id);
Trame PiloteServoDemandeModele(char id);
Trame PiloteServoDemandePositionActuelle(char id);
Trame PiloteServoDemandePositionCible(char id);
Trame PiloteServoDemandePositionMax(char id);
Trame PiloteServoDemandePositionMin(char id);
Trame PiloteServoDemandeTemperature(char id);
Trame PiloteServoDemandeTension(char id);
Trame PiloteServoDemandeVersionFirmware(char id);
Trame PiloteServoDemandeVitesseActuelle(char id);
Trame PiloteServoDemandeVitesseMax(char id);
Trame PiloteServoDemandeTemperatureMax(char id);
Trame PiloteServoDemandeCoupleLimitMax(char id);
Trame PiloteServoDemandeAllIn(char id);

int PiloteServoEnvoiAlarmeLED(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction);
int PiloteServoEnvoiAlarmeShutdown(char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction);
int PiloteServoEnvoiComplianceParams(char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin);
int PiloteServoEnvoiCoupleActive(char id, char coupleActive);
int PiloteServoEnvoiCoupleMaximum(char id, unsigned int coupleMax);
int PiloteServoEnvoiId(char id, char nouvelId);
int PiloteServoEnvoiLed(char id, char ledAllume);
int PiloteServoEnvoiTensionMax(char id, unsigned int TensionMax);
int PiloteServoEnvoiTensionMin(char id, unsigned int TensionMin);
int PiloteServoEnvoiTemperatureMax(char id, unsigned int TemperatureMax);
int PiloteServoEnvoiPosistionCible(char id, unsigned int positionCible);
int PiloteServoEnvoiPosistionMax(char id, unsigned int positionMax);
int PiloteServoEnvoiPosistionMin(char id, unsigned int positionMin);
int PiloteServoEnvoiVitesseMax(char id, unsigned int vitesseMax);
int PiloteServoReset(char id);
int PiloteServoEnvoiCoupleLimitMax(char id, unsigned int coupleLimitMax);
int PiloteServoEnvoiBaudrate(char id, unsigned char newBaud);

// DEBUG
Trame PiloteDebug0(Trame t);
Trame PiloteDebug1(Trame t);
Trame PiloteDebug2(Trame t);
Trame PiloteDebug3(Trame t);
Trame PiloteDebug4(Trame t);
Trame PiloteDebug5(Trame t);
Trame PiloteDebug6(Trame t);
Trame PiloteDebug7(Trame t);
Trame PiloteDebug8(Trame t);
Trame PiloteDebug9(Trame t);

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
#define CMD_STOP_MOTEUR					0x69

// Debug asservissement
#define CMD_DEMANDE_BUFF_POSITION		0x43
#define CMD_REPONSE_BUFF_POSITION		0x44
#define CMD_CONSIGNE_POSITION			0x45
#define CMD_DEMANDE_BUFF_STATUS			0x46
#define CMD_REPONSE_BUFF_STATUS			0x47

// Autre
#define CMD_SERVOMOTEUR					0x60
#define CMD_ENVOI_BAUDRATE_MICRO		0x61

// Actionneurs
#define CMD_MOTEUR_ORIGIN				0x63
#define CMD_MOTEUR_INIT					0x64
#define TRAME_PILOTAGE_ONOFF 			0x65
#define CMD_MOTEUR_POSITION				0x66
#define CMD_VITESSE_MOTEUR				0x67
#define CMD_ACCELERATION_MOTEUR			0x68
#define ID_MOTEUR_ASCENSEUR_GAUCHE		0x00
#define ID_MOTEUR_ASCENSEUR_DROITE		0x01
#define ID_MOTEUR_BALISE				0x02
#define ID_MOTEUR_POMPE					0x03
#define CMD_DEMANDE_CALIBRATION_ASCENSEUR_AMPOULE 0x79

// Capteurs onOff
#define CMD_DEMANDE_CAPTEUR_ONOFF		0x74
#define CMD_REPONSE_CAPTEUR_ONOFF		0x75
#define ID_SWITCH_BRAS_DROIT_HAUT		0x00
#define ID_SWITCH_BRAS_DROIT_BAS		0x01
#define ID_BARRIERE_BRAS_DROIT			0x02
#define ID_SWITCH_BRAS_GAUCHE_HAUT		0x03
#define ID_SWITCH_BRAS_GAUCHE_BAS		0x04
#define ID_BARRIERE_BRAS_GAUCHE			0x05
#define ID_SWITCH_ORIGINE_BRAS_DROIT	0x06
#define ID_SWITCH_ORIGINE_BRAS_GAUCHE	0x07
#define CMD_DEMANDE_CAPTEUR				0x50
#define CMD_REPONSE_CAPTEUR				0x51
#define ID_CAPTEUR_BALISE 				0x01
#define ID_CAPTEUR_BALISE_1				0x02
#define ID_CAPTEUR_BALISE_2				0x03
#define CMD_DEMANDE_LIDAR				0x10
#define CMD_REPONSE_LIDAR				0x11


// Servomoteurs
#define CMD_SERVO_DEMANDE_POSITION_CIBLE		0x01
#define CMD_SERVO_RETOUR_POSITION_CIBLE			0x02
#define CMD_SERVO_ENVOI_POSITION_CIBLE			0x03
#define CMD_SERVO_ENVOI_BAUDRATE				0x06
#define CMD_SERVO_DEMANDE_VITESSE_MAX			0x07
#define CMD_SERVO_RETOUR_VITESSE_MAX			0x08
#define CMD_SERVO_ENVOI_VITESSE_MAX				0x09
#define CMD_SERVO_DEMANDE_ALL_IN				0x10
#define CMD_SERVO_RETOUR_ALL_IN					0x11
#define CMD_SERVO_ENVOI_ID						0x12
#define CMD_SERVO_RESET							0x13
#define CMD_SERVO_DEMANDE_COUPLE_MAX			0x14
#define CMD_SERVO_RETOUR_COUPLE_MAX				0x15
#define CMD_SERVO_ENVOI_COUPLE_MAX				0x16
#define CMD_SERVO_DEMANDE_COUPLE_ACTIVE			0x17
#define CMD_SERVO_RETOUR_COUPLE_ACTIVE			0x18
#define CMD_SERVO_ENVOI_COUPLE_ACTIVE			0x19
#define CMD_SERVO_DEMANDE_TENSION				0x20
#define CMD_SERVO_RETOUR_TENSION				0x21
#define CMD_SERVO_DEMANDE_TEMPERATURE			0x22
#define CMD_SERVO_RETOUR_TEMPERATURE			0x23
#define CMD_SERVO_DEMANDE_MOUVEMENT				0x24
#define CMD_SERVO_RETOUR_MOUVEMENT				0x25
#define CMD_SERVO_DEMANDE_POSITION_MIN			0x26
#define CMD_SERVO_RETOUR_POSITION_MIN			0x27
#define CMD_SERVO_ENVOI_POSITION_MIN			0x28
#define CMD_SERVO_DEMANDE_POSITION_MAX			0x29
#define CMD_SERVO_RETOUR_POSITION_MAX			0x30
#define CMD_SERVO_ENVOI_POSITION_MAX			0x31
#define CMD_SERVO_DEMANDE_NUMERO_MODELE			0x32
#define CMD_SERVO_RETOUR_NUMERO_MODELE			0x33
#define CMD_SERVO_DEMANDE_VERSION_FIRMWARE		0x34
#define CMD_SERVO_RETOUR_VERSION_FIRMWARE		0x35
#define CMD_SERVO_DEMANDE_LED					0x36
#define CMD_SERVO_RETOUR_LED					0x37
#define CMD_SERVO_ENVOI_LED						0x38
#define CMD_SERVO_DEMANDE_CFG_ALARME_LED		0x42
#define CMD_SERVO_RETOUR_CFG_ALARME_LED			0x43
#define CMD_SERVO_ENVOI_CFG_ALARME_LED			0x44
#define CMD_SERVO_DEMANDE_CFG_ALARME_SHUTDOWN	0x45
#define CMD_SERVO_RETOUR_CFG_ALARME_SHUTDOWN	0x46
#define CMD_SERVO_ENVOI_CFG_ALARME_SHUTDOWN		0x47
#define CMD_SERVO_DEMANDE_CFG_ECHO				0x48
#define CMD_SERVO_RETOUR_CFG_ECHO				0x49
#define CMD_SERVO_ENVOI_CFG_ECHO				0x50
#define CMD_SERVO_DEMANDE_COMPLIANCE_PARAMS		0x51
#define CMD_SERVO_RETOUR_COMPLIANCE_PARAMS		0x52
#define CMD_SERVO_ENVOI_COMPLIANCE_PARAMS		0x53
#define CMD_SERVO_DEMANDE_POSITION_ACTUELLE		0x54
#define CMD_SERVO_RETOUR_POSITION_ACTUELLE		0x55
#define CMD_SERVO_DEMANDE_VITESSE_ACTUELLE		0x56
#define CMD_SERVO_RETOUR_VITESSE_ACTUELLE		0x57
#define CMD_SERVO_DEMANDE_ERREURS				0x58
#define CMD_SERVO_RETOUR_ERREURS				0x59
#define CMD_SERVO_DEMANDE_COUPLE_COURANT        0x60
#define CMD_SERVO_RETOUR_COUPLE_COURANT         0x61
#define CMD_SERVO_DEMANDE_STATUS_LEVEL			0X62
#define CMD_SERVO_RETOUR_STATUS_LEVEL 			0x63
#define CMD_SERVO_ENVOI_TENSION_MAX				0x64
#define CMD_SERVO_DEMANDE_TENSION_MAX			0x65
#define CMD_SERVO_RETOUR_TENSION_MAX			0x66
#define CMD_SERVO_ENVOI_TENSION_MIN				0x67
#define CMD_SERVO_DEMANDE_TENSION_MIN			0x68
#define CMD_SERVO_RETOUR_TENSION_MIN			0x69
#define CMD_SERVO_ENVOI_TEMPERATURE_MAX 		0x70
#define CMD_SERVO_DEMANDE_TEMPERATURE_MAX		0x71
#define CMD_SERVO_RETOUR_TEMPERATURE_MAX		0x72
#define CMD_SERVO_ENVOI_COUPLE_LIMIT_MAX		0x73
#define CMD_SERVO_DEMANDE_COUPLE_LIMIT_MAX		0x74
#define CMD_SERVO_RETOUR_COUPLE_LIMIT_MAX		0x75

// Passerelle UART
#define CMD_ENVOI_UART					0xA0
#define CMD_RECEPTION_UART				0xA1
#define CMD_ENVOI_UART2					0xA4
#define CMD_RECEPTION_UART2				0xA5


// Capteurs
#define CMD_DEPART_JACK					0x71
#define CMD_DEMANDE_COULEUR_EQUIPE		0x72
#define CMD_REPONSE_COULEUR_EQUIPE		0x73
#define CMD_ARME_JACK					0x70
#define CMD_DEMANDE_VALEURS_ANALOGIQUES	0x76
#define CMD_REPONSE_VALEURS_ANALOGIQUES	0x77
#define CMD_DETECTION_FRONT_ANALOGIQUES	0x78
#define CMD_DETECTION_SECU_ASCENSEUR	0x82

// Alimentation
#define CMD_ALIMENTATION				0x80

// Diagnostic
#define	CMD_DEBUG						0xEE
#define	CMD_DEMANDE_ECHO				0xF0
#define	CMD_RESET_CARTE					0xF1
#define CMD_DEMANDE_PRESENCE_JACK		0xF3
#define CMD_REPONSE_PRESENCE_JACK		0xF4
#define CMD_REPONSE_ECHO				0xF0


// Liste des actionneurs
#define ALIMENTATION_CAPTEUR_COULEUR	0x01

#endif // __PILOTAGE_H__
