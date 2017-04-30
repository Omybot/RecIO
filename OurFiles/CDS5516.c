#include <p33FJ128MC804.h>
#include "CDS5516.h"
#include "FonctionsUc.h"

extern unsigned char trame_recu[20],flag_servo,recu_nbr,timeout_servo;

int globalBaudrate = UXBRG_AX12_19200;

/////////////////////////////////////////////////
//                                             //
//              Communication Servo            //
//                                             //
/////////////////////////////////////////////////
//delay ajouté par Mouly
void delay_us_uart(float mult){
    long i = 40000*mult;
    while(i--);
}

void CDS5516SetUXBRG(int baud)
{
	globalBaudrate = baud;
}

// Envoi d'un message série au servomoteur
void CDS5516EnvoiMessage(char baud, char id, char fonction, int valeur, char nbChar)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	int i;

	DIRECTION = 1;	// 1 J'envoie et 0 je réceptionne

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U1BRG = globalBaudrate;	
												
	//Instruction écriture valeur
	instruction = 0x03;  	
	adresse = fonction; 
	if(nbChar == 2)
	{	
		taille = 0x05; 
		val = valeur & 0b0000000011111111;
		val1 = valeur >> 8;
		checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));
	}
	else if(nbChar == 1)
	{
		taille = 0x04; 
		val = valeur;
		checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val));
	}

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(adresse);
	UART2PutChar(val);	
	if(nbChar == 2)
		UART2PutChar(val1);
	UART2PutChar(checksum);
}

// Envoi d'un ordre série de reset au servomoteur
void CDS5516Reset(char baud, char id)
{
	char taille = 0, instruction = 0, checksum = 0;
	int i;

	DIRECTION = 1;	// 1 J'envoie et 0 je réceptionne

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U1BRG = globalBaudrate; 

	taille = 0x02;
	instruction = 0x06;  
	checksum = (char)(0xFF-(char)(id + taille + instruction));

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(checksum);
}

// Envoi d'un ping série au servomoteur auquel il est censé répondre par un état de ses erreurs
int CDS5516Ping(char baud, char id)
{
	char taille = 0, checksum = 0, instruction = 0;
	int i,valeurRecue;
	char checksumRecu;
	char messageRecu[20];

	DIRECTION = 1;	// 1 J'envoie et 0 je réceptionne

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U1BRG = globalBaudrate;
 
	taille = 0x02;  															//Nbr de paramètres + 2
	instruction = 0x01; 
	checksum = (char)(0xFF-(char)(id + taille + instruction));

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(checksum);

	// Réception de la réponse
	DIRECTION = 0;	// 1 J'envoie et 0 je réceptionne

	timeout_servo = 0;
	while(flag_servo==0) if(timeout_servo>10) return(recu_nbr); // ajouter un timeout !!!!
	flag_servo=0;
	for(i=0;i<20;i++)
		messageRecu[i] = trame_recu[i];
	i=recu_nbr+4;
	checksum -= messageRecu[i - 1];
	checksum = (char)(0xFF-(char)(checksum));

	//if(messageRecu[0] != 0xFF || messageRecu[1] != 0xFF)// || messageRecu[2] != id || messageRecu[3] != 2 || checksum != messageRecu[i - 1])
	//	return 42;

	valeurRecue = messageRecu[i - 2];
	return valeurRecue;
}

// Envoi un message série au servomoteur qui demande un retour qui est une lecture d'une valeur registre
int CDS5516DemandeMessage(char baud, char id, char fonction, char nbChar)
{
	char taille = 0, instruction = 0, adresse = 0,checksum = 0;
	int i, j,valeurRecue;
	char checksumRecu;
	char messageRecu[20];

	DIRECTION = 1;	// 1 J'envoie et 0 je réceptionne

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U1BRG = globalBaudrate; 

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x02;  														//Instruction écriture valeur
	adresse = fonction;
	checksum = (char)(0xFF-(char)(id + taille + instruction + nbChar + adresse));

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(adresse);
	UART2PutChar(nbChar);
	UART2PutChar(checksum);

	// Réception de la réponse
	DIRECTION = 0;	// 1 J'envoie et 0 je réceptionne
	
	// Ici il faut que messageRecu contienne la trame servo recu
	timeout_servo = 0;
	while(flag_servo==0) if(timeout_servo>10) return(42); // ajouter un timeout !!!!
	flag_servo=0;
	for(i=0;i<20;i++)
		messageRecu[i] = trame_recu[i];
	i=recu_nbr+4;
	checksum -= messageRecu[i - 1];
	checksum = (char)(0xFF-(char)(checksum));

	//if(messageRecu[0] != 0xFF || messageRecu[1] != 0xFF || messageRecu[2] != id )//|| messageRecu[3] != 2 )//|| checksum != messageRecu[i - 1])
	//	return 42;

	if(nbChar == 1)
	{
		valeurRecue = messageRecu[i - 2];
		return valeurRecue;
	}
	else
	{
		valeurRecue  = (int)messageRecu[i - 2];
		valeurRecue  = valeurRecue<<8;
		valeurRecue  = valeurRecue+(((int)(messageRecu[i - 3]))&0x00FF); // canard en plastique d'or
		return valeurRecue;
	}
}

// Envoi un message série au servomoteur qui demande un retour qui est une lecture d'une valeur registre
char* CDS5516DemandeMessageAllIn(char id, char fonction)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0, nbChar = NB_CHAR_ALL_IN;
	int i, j;
	char checksumRecu;
	static char messageRecu[TAILLE_MSG_UART];
	char tabError[2] = {42,42};

	DIRECTION = 1;	// 1 J'envoie et 0 je réceptionne

	U1BRG = globalBaudrate; 

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x02;  														//Instruction écriture valeur
	adresse = fonction;
	checksum = (char)(0xFF-(char)(id + taille + instruction + nbChar + adresse));

	UART2PutChar(0xFF);
	UART2PutChar(0xFF);
	UART2PutChar(id);
	UART2PutChar(taille);
	UART2PutChar(instruction);
	UART2PutChar(adresse);
	UART2PutChar(nbChar);
	UART2PutChar(checksum);

	// Réception de la réponse
	DIRECTION = 0;	// 1 J'envoie et 0 je réceptionne
	
	// Ici il faut que messageRecu contienne la trame servo recu
	timeout_servo = 0;
	while(flag_servo==0) if(timeout_servo>10) return tabError; // ajouter un timeout !!!!
	flag_servo=0;
	for(i = 0; i < TAILLE_MSG_UART; i++)
		messageRecu[i] = trame_recu[i];

	i = recu_nbr+4;
	checksum -= messageRecu[i - 1];
	checksum = (char)(0xFF-(char)(checksum));

	//if(messageRecu[0] != 0xFF || messageRecu[1] != 0xFF || messageRecu[2] != id )//|| messageRecu[3] != 2 )//|| checksum != messageRecu[i - 1])
	//	return 42;
	return messageRecu;
}

/////////////////////////////////////////////////
//                                             //
//       Fonctions de demande de valeurs       //
//                                             //
/////////////////////////////////////////////////

int CDS5516DemandeCCWMargin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1B, 1);
}

int CDS5516DemandeCWMargin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1A, 1);
}

int CDS5516DemandeCCWSlope(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1D, 1);
}

int CDS5516DemandeCWSlope(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1C, 1);
}

int CDS5516DemandeConfigAlarmeLED(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x11, 1);
}

int CDS5516DemandeConfigAlarmeShutdown(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x12, 1);
}

int CDS5516DemandeConfigEcho(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x10, 1);
}

int CDS5516DemandeCoupleActive(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x18, 1);
}

int CDS5516DemandeCoupleCourant(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x28, 2);
}

int CDS5516DemandeCoupleMaximum(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x0E, 2);
}

int CDS5516DemandeLed(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x19, 1);
}

int CDS5516DemandeMouvement(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x2E, 1);
}

int CDS5516DemandeModele(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x00, 1);
}

int CDS5516DemandePositionActuelle(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x24, 2);
}

int CDS5516DemandePositionCible(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x1E, 2);
}

int CDS5516DemandePositionMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x08, 2);
}

int CDS5516DemandePositionMin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x06, 2);
}

int CDS5516DemandeCoupleLimitMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x22, 2);
}

int CDS5516DemandeTemperature(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x2B, 1);
}

int CDS5516DemandeTension(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x2A, 1);
}

int CDS5516DemandeVersionFirmware(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x02, 1);
}

int CDS5516DemandeVitesseActuelle(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x26, 2);
}

int CDS5516DemandeVitesseMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x20, 1);
}

int CDS5516DemandeStatusReturnLevel(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x10, 1);
}

int CDS5516DemandeTensionMin(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x0C, 1);
}

int CDS5516DemandeTensionMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x0D, 1);
}

int CDS5516DemandeTemperatureMax(char baud, char id)
{
	return CDS5516DemandeMessage(baud, id, 0x0B, 1);
}

/////////////////////////////////////////////////
//                                             //
//      Fonctions envoi valeur servo           //
//                                             //
/////////////////////////////////////////////////

//Contrôle des paramètres de compliance
void CDS5516EnvoiComplianceParams(char baud, char id, char CCWSlope, char CCWMargin, char CWSlope, char CWMargin)
{
	CDS5516EnvoiMessage(baud, id, 0x1A, CWMargin, 1);
	CDS5516EnvoiMessage(baud, id, 0x1B, CCWMargin, 1);
	CDS5516EnvoiMessage(baud, id, 0x1C, CWSlope, 1);
	CDS5516EnvoiMessage(baud, id, 0x1D, CCWSlope, 1);
}

//Contrôle de la configuration du déclenchement de l'alarme Shutdown
void CDS5516EnvoiAlarmeShutdown(char baud, char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	int valeur = 0;
	if(inputVoltage)
		valeur = valeur | 0b00000001;
	if(angleLimit)
		valeur = valeur | 0b00000010;
	if(overheating)
		valeur = valeur | 0b00000100;
	if(range)
		valeur = valeur | 0b00001000;
	if(checksum)
		valeur = valeur | 0b00010000;
	if(overload)
		valeur = valeur | 0b00100000;
	if(instruction)
		valeur = valeur | 0b01000000;

	CDS5516EnvoiMessage(baud, id, 0x12, valeur, 1);
}

//Contrôle de la configuration du déclenchement de l'alarme LED
void CDS5516EnvoiAlarmeLED(char baud, char id, char inputVoltage, char angleLimit, char overheating, char range, char checksum, char overload, char instruction)
{
	int valeur = 0;
	if(inputVoltage)
		valeur = valeur | 0b00000001;
	if(angleLimit)
		valeur = valeur | 0b00000010;
	if(overheating)
		valeur = valeur | 0b00000100;
	if(range)
		valeur = valeur | 0b00001000;
	if(checksum)
		valeur = valeur | 0b00010000;
	if(overload)
		valeur = valeur | 0b00100000;
	if(instruction)
		valeur = valeur | 0b01000000;

	CDS5516EnvoiMessage(baud, id, 0x11, valeur, 1);
}

//Contrôle de la position minimum de 0 à 1023
void CDS5516EnvoiPositionMin(char baud, char id, unsigned int positionMin)
{
	CDS5516EnvoiMessage(baud, id, 0x06, positionMin, 2);
}

//Contrôle de la réponse retournée à chaque message reçu
void CDS5516EnvoiConfigEcho(char baud, char id, char config)
{
	CDS5516EnvoiMessage(baud, id, 0x10, config, 1);
}

//Contrôle de la position maximum de 0 à 1023
void CDS5516EnvoiPositionMax(char baud, char id, unsigned int positionMax)
{
	CDS5516EnvoiMessage(baud, id, 0x08, positionMax, 2);
}

//Contrôle en angle du cds5516 (0 à 1023 soit 0° à 300°)
void CDS5516EnvoiPosistionCible(char baud, char id, unsigned int positionCible)
{
	CDS5516EnvoiMessage(baud, id, 0x1E, positionCible, 2);
}

//Paramètre de la vitesse du CDS5516 (0 à 1023)
void CDS5516EnvoiVitesseMax(char baud, char id, unsigned int vitesseMax)
{
	CDS5516EnvoiMessage(baud, id, 0x20, vitesseMax, 2);
}

//Allumer la led du CDS5516
void CDS5516EnvoiLed(char baud, char id, char ledAllume)
{
	CDS5516EnvoiMessage(baud, id, 0x19, ledAllume, 1);
}

//Activation du couple du CDS5516 (0 ou 1)
void CDS5516EnvoiCoupleActive(char baud, char id, char enableTorque)
{
	CDS5516EnvoiMessage(baud, id, 0x18, enableTorque, 1);
}

//Changer le bauderate du CDS5516 
void CDS5516EnvoiBauderate(char baud, char id, char finalBauderate)
{
	CDS5516EnvoiMessage(baud, id, 0x04, finalBauderate, 1);
}

//Changer l'ID du CDS5516 
void CDS5516EnvoiId(char baud, char id, char nouvelId)
{
	CDS5516EnvoiMessage(baud, id, 0x03, nouvelId, 1);
}

//Régler le maximum de couple en EEPROM (0 à 1023)
void CDS5516EnvoiCoupleMaximum(char baud, char id, unsigned int coupleMax)
{
	CDS5516EnvoiMessage(baud, id, 0x0E, coupleMax, 2);
}

//Régler le maximum de couple limite en EEPROM (0 à 1023)
void CDS5516EnvoiCoupleLimitMax(char baud, char id, unsigned int coupleLimitMax)
{
	CDS5516EnvoiMessage(baud, id, 0x22, coupleLimitMax, 2);
}

//Régler le maximum de la tension en EEPROM (50 à 250)
void CDS5516EnvoiTensionMax(char baud, char id, unsigned int TensionMax)
{
	CDS5516EnvoiMessage(baud, id, 0x0D, TensionMax, 1);
}

//Régler le minimum de la tension en EEPROM (50 à 250)
void CDS5516EnvoiTensionMin(char baud, char id, unsigned int TensionMin)
{
	CDS5516EnvoiMessage(baud, id, 0x0C, TensionMin, 1);
}

//Régler le maximum de la temperature en EEPROM (0 à 150)
void CDS5516EnvoiTemperatureMax(char baud, char id, unsigned int TemperatureMax)
{
	CDS5516EnvoiMessage(baud, id, 0x0B, TemperatureMax, 1);
}

//Régler le baud rate en EEPROM :
// Bauderate = 1   pour 1000000
// Bauderate = 3   pour 500000
// Bauderate = 4   pour 400000
// Bauderate = 7   pour 250000
// Bauderate = 9   pour 200000
// Bauderate = 16  pour 115200
// Bauderate = 34  pour 57600
// Bauderate = 103 pour 19200
// Bauderate = 207 pour 9600
void CDS5516EnvoiBaudrate(char baud, char id, unsigned char newBaud)
{
	CDS5516EnvoiMessage(baud, id, 0x04, newBaud, 1);
}
