#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/UDPPerformanceTest.h"
#include "Main804.h"
#include "OurFiles/UserUdp.h"
#include "OurFiles/asser.h"
#include "OurFiles/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "OurFiles/Pilotage.h"
#include "OurFiles/CDS5516.h"
#include "OurFiles/FonctionsUc.h"
#include "OurFiles/Stepper.h"

#define ANTIREBOND 50

#define IDCAPTEUR_BAS 0
#define IDCAPTEUR_HAUT 1

#define UART_BUFFER_SIZE	100

unsigned char Demande_lidar = 0;

unsigned char timeout_lidar=0;
unsigned char nbr_char_to_send=0,Buffer_passerelle_udpuart[250];
unsigned char ptr_write_buffer_uart_rec=0,save_write;
unsigned char ptr_read_buffer_uart_rec=0,save_read;

unsigned char cpt_antirebond_rb3,old_rb3;

unsigned char flag_envoi_uart,buffer_envoi_uart[UART_BUFFER_SIZE],ptr_write_buffer_uart;
unsigned char ptr_read_buffer_uart=0,ptr_write_buffer_uart=0;
unsigned char buffer_envoi_uart[UART_BUFFER_SIZE];

unsigned char jack_cpt=0;
unsigned char securite_ascenseur=0;
extern unsigned char telem,telem_front;
unsigned int periode_tour;
char Fin_de_tour, Fin_d_angle_bas = 0,Fin_d_angle_haut = 0;
char capteurHautPrec = 0;
char capteurBasPrec = 0;
char oldMagnet = 0;
unsigned char nombre_angles[2]; // n angles (8 maxi)
unsigned char buffer_angles[2][2*8]; // debut_1_MSB; debut_1_LSB; fin_1_MSB; fin_1_LSB; debut_2_MSB; debut_2_LSB; fin_2_MSB; fin_2_LSB; ...;debut_n_MSB; debut_n_LSB; fin_n_MSB; fin_n_LSB; 
unsigned char nombre_fronts[2]; // n fronts (8 maxi)
unsigned int buffer_fronts[2][8]; // Valeurs bruts TMR2
unsigned int buffer_fronts_temp[2][8]; // Valeurs bruts TMR2
unsigned int front_rapide[2][2];
unsigned int ptr_fronts_bas; // ptr pour l'enregistrement
unsigned int ptr_fronts_haut; // 
unsigned int hall_front;
unsigned int motor_speed,Cpt_20ms=0;
float angle;

extern double targ_pos[2];
extern unsigned int ADC_Results[9];
extern unsigned int position_ascenseur_gauche,position_ascenseur_droite;

unsigned char Coo_SwitchBrasDroitHaut;
unsigned char Coo_SwitchBrasDroitBas;
unsigned char Coo_BarriereBrasDroit;
unsigned char Coo_SwitchBrasGaucheHaut;
unsigned char Coo_SwitchBrasGaucheBas;
unsigned char Coo_BarriereBrasGauche;
unsigned char Coo_SwitchOrigineBrasDroit;
unsigned char Coo_SwitchOrigineBrasGauche;
unsigned char Old_Coo_SwitchBrasDroitHaut;
unsigned char Old_Coo_SwitchBrasDroitBas;
unsigned char Old_Coo_BarriereBrasDroit;
unsigned char Old_Coo_SwitchBrasGaucheHaut;
unsigned char Old_Coo_SwitchBrasGaucheBas;
unsigned char Old_Coo_BarriereBrasGauche;
unsigned char Old_Coo_SwitchOrigineBrasDroit;
unsigned char Old_Coo_SwitchOrigineBrasGauche;
unsigned char SwitchsStatusGauche,OldSwitchsStatusGauche,SwitchsStatusGaucheCpt;
unsigned char SwitchsStatusDroit,OldSwitchsStatusDroit,SwitchsStatusDroitCpt;
double Mesure;
double SeuilsSwitchs[15] = {1.519,1.577,1.64,1.705,1.775,1.855,1.942,2.061,2.191,2.316,2.452,2.601,2.768,2.969,3.97};

Trame Etat_Capteur_OnOff;
static BYTE messCapteur_OnOff[4];


// TODO list
// * Self-calibration de l'odométrie
// * Self-check de l'asservissement avant chaque début de match
// * Self-placement du robot
// * Procédure de calibration odométrique auto
// * Asservissement polaire

// Bits configuration
_FOSCSEL(FNOSC_PRIPLL)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_EC)
_FPOR(FPWRT_PWR1)
_FWDT(FWDTEN_OFF)// & SWDTEN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

float vbat[2];

APP_CONFIG AppConfig;

static void InitAppConfig(void);
extern unsigned int courant;
extern unsigned int ADC_Results[9],cpu_status;
extern double cons_pos[N];
extern double real_pos[N];
extern unsigned char scan;

unsigned char jackAvant = 0,trame_recu[TAILLE_MSG_UART],recu_nbr,timeout_servo;
unsigned char motor_flag=0,datalogger_blocker=0;
double position_lock;
unsigned int datalogger_counter=0,flag=0,courrier=0,PID_ressource_used;
unsigned char flag_envoi=0,flag_blocage=0,flag_calage=0,flag_servo=0,detection_pompe=0;

long lcourant;
unsigned int cpt_20mscourant;

//Variable Pwm_Servos_Timer2
unsigned int Cpt_Timer4 = 0;

//Variable Capteur de vitesse
unsigned char flag_capteur_vitesse;
double vitesse_canon;
unsigned int consigne_canon;
unsigned int cpt_capteur_vitesse,capteur_vitesse;
unsigned char desactive_interrupt;
unsigned int prd_asser_canon=50,electrovanne=0;

unsigned char tir=0,tir_power=0;

void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
{
    Nop();
	Nop();
}
void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
{
    Nop();
	Nop();
}

int main(void)
{
	
	unsigned char etatCouleur = 2,i;
	static DWORD dwLastIP = 0;
	int indice_test = 0;
	
	Trame trame;		

	Trame Jack;
	static BYTE Presence[2];
	Jack.nbChar = 2;
	Presence[0] = 0xC4;
	Presence[1] = CMD_DEPART_JACK;
	Jack.message = Presence;

	Trame Couleur_Equipe;
	static BYTE Couleur[3];
	Couleur_Equipe.nbChar = 3;
	Couleur[0] = 0xC4;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = MATCH_COULEUR;
	Couleur_Equipe.message = Couleur;
	
	Trame envoiFin;
	static BYTE mess[2];
	mess[0] = 0xC4;
	mess[1] = CMD_FINDEPLACEMENT;
	envoiFin.message = mess;
	envoiFin.nbChar = 2;
		
	Trame envoiBlocage;
	static BYTE messblocage[2];
	messblocage[0] = 0xC4;
	messblocage[1] = CMD_BLOCAGE;
	envoiBlocage.message = messblocage;
	envoiBlocage.nbChar = 2;
		
	Trame envoiBalise;
	static BYTE messbalise[60];
	messbalise[0] = 0xC4;
	messbalise[1] = CMD_REPONSE_CAPTEUR;
	messbalise[2] = ID_CAPTEUR_BALISE;
	envoiBalise.message = messbalise;
	envoiBalise.nbChar = 30;

	Trame envoiBaliserapide;
	static BYTE messbaliserapide[7];
	messbaliserapide[0] = 0xC4;
	messbaliserapide[1] = CMD_REPONSE_CAPTEUR;
	envoiBaliserapide.message = messbaliserapide;
	envoiBaliserapide.nbChar = 7;
	
	Trame envoiGP2;
	static BYTE messGP2[4];
	messGP2[0] = 0xC4;
	messGP2[1] = CMD_DETECTION_FRONT_ANALOGIQUES;
	envoiGP2.message = messGP2;
	envoiGP2.nbChar = 4;
	
	Trame envoiSecuAsc;
	static BYTE messSecuAsc[3];
	messSecuAsc[0] = 0xC4;
	messSecuAsc[1] = CMD_DETECTION_SECU_ASCENSEUR;
	envoiSecuAsc.message = messSecuAsc;
	envoiSecuAsc.nbChar = 3;
	
	Trame envoiUART;
	static BYTE messUART[250];
	messUART[0] = 0xC4;
	messUART[1] = CMD_REPONSE_LIDAR;//CMD???;
	messUART[2] = 0xFE;
	envoiUART.message = messUART;
	envoiUART.nbChar = 53;
	
	
	Trame envoiTest;
	static BYTE messTest[19];
	messTest[0] = 0xC4;
	messTest[1] = 0xC4;
	messTest[2] = 7;
	messTest[3] = 'M';
	messTest[4] = 'S';
	messTest[5] = '0';
	messTest[6] = '0';
	messTest[7] = '0';
	messTest[8] = '0';
	messTest[9] = '0';
	messTest[10] = '7';
	messTest[11] = '2';
	messTest[12] = '5';
	messTest[13] = '0';
	messTest[14] = '0';
	messTest[15] = '0';
	messTest[16] = '0';
	messTest[17] = '1';
	messTest[18] = '\n';

	//MS0000072500001
	
	envoiTest.message = messTest;
	envoiTest.nbChar = 19;
	// V V [LF] 0 0 P [LF] 
	
	Etat_Capteur_OnOff.nbChar = 4;
	messCapteur_OnOff[0] = 0xC4;
	messCapteur_OnOff[1] = CMD_REPONSE_CAPTEUR_ONOFF;
	Etat_Capteur_OnOff.message = messCapteur_OnOff;
	
	InitClk(); 		// Initialisation de l'horloge
	InitPorts(); 	// Initialisation des ports E/S

	// Initialize the Input Capture Module
	IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC1CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
	IC1CONbits.ICI = 0b00; // Interrupt on every capture event
	IC1CONbits.ICM = 0b011; // Generate capture event on every Rising edge
	// Enable Capture Interrupt And Timer2
	IPC0bits.IC1IP = 3; // Setup IC1 interrupt priority level
	IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
	IEC0bits.IC1IE = 1; // Enable IC1 interrupt
	// Initialize the Input Capture Module
	IC2CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC2CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
	IC2CONbits.ICI = 0b00; // Interrupt on every capture event
	IC2CONbits.ICM = 0b011; // Generate capture event on every edge // change
	// Enable Capture Interrupt And Timer2
	IPC1bits.IC2IP = 2; // Setup IC1 interrupt priority level
	IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
	IEC0bits.IC2IE = 1; // Enable IC1 interrupt
	// Initialize the Input Capture Module
	IC7CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC7CONbits.ICTMR = 1; // Select Timer2 as the IC1 Time base
	IC7CONbits.ICI = 0b00; // Interrupt on every capture event
	IC7CONbits.ICM = 0b011; // Generate capture event on every edge // change
	// Enable Capture Interrupt And Timer2
	IPC5bits.IC7IP = 1; // Setup IC1 interrupt priority level
	IFS1bits.IC7IF = 0; // Clear IC1 Interrupt Status Flag
	IEC1bits.IC7IE = 1; // Enable IC1 interrupt
	
	Init_Timer();	// Initialisation Timer2,Timer4 & Timer5
	InitQEI(); 		// Initialisation des entrées en quadrature
	InitPWM();		// Configuration du module PWM 
	//InitStepper();	// Init Stepper Ascenseur arriere (CNInterrupt et Timer5)

	pwm(ID_MOTEUR_BALISE,0);
	pwm(ID_MOTEUR_ASCENSEUR_GAUCHE,0);
	pwm(ID_MOTEUR_ASCENSEUR_DROITE,0);
	pwm(ID_MOTEUR_POMPE,0);	
	

	//DelayMs(100); // Necessaire car InitProp a besoin de valeurs ADC valides	
	InitProp();
	
	Motors_Power(OFF,0);
	Motors_Power(OFF,1);
	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
    TickInit();
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		MPFSInit();
	#endif
	
	// Initialize Stack and application related NV variables into AppConfig.
	InitAppConfig();
	
	UDPInit();
    StackInit();	
 	UDPPerformanceTask();
	InitUserUdp();
	
	Init_Interrupt_Priority();							
	InitUART1();	
	Init_Servos();
	//Init_Input_Capture();
//	Init_Alimentation();
	
	InitADC();
	InitDMA();
	DelayMs(100); 
	position_ascenseur_droite=1;
	position_ascenseur_gauche=1;
	Go(3000,0);
	Go(1300,1);
	Motors_Power(ON,0);
	Motors_Power(ON,1);
	
	pwm(ID_MOTEUR_ASCENSEUR_GAUCHE,0); // BRAS DROIT sens - pour prendre
	pwm(ID_MOTEUR_ASCENSEUR_DROITE,0);	// BRAS GAUCHE sens - pour prendre

	// Test AX12
	CDS5516EnvoiLed(103,17,0);
	DelayMs(1000); 
	CDS5516EnvoiLed(103,17,1);
	DelayMs(1000); 
	CDS5516EnvoiLed(103,17,0);
	DelayMs(1000); 
	CDS5516EnvoiLed(103,17,1);

	while(1)
  	{
		if(Demande_lidar)	
		{
			Demande_lidar=0;
			EnvoiUART(envoiTest);
		}
		
		save_write = ptr_write_buffer_uart_rec;
		save_read = ptr_read_buffer_uart_rec;
		if((save_write != save_read))
		{
			if(save_write < save_read)
				nbr_char_to_send = 241 - save_read + save_write;
			else
				nbr_char_to_send = save_write - save_read;
		}
		else
		{
			nbr_char_to_send = 0;
		}	

		if(((nbr_char_to_send > 200) || (nbr_char_to_send !=0 && timeout_lidar > 50)))
		{	
			for(i=0;i<nbr_char_to_send;i++)
			{
				messUART[i+3]=Buffer_passerelle_udpuart[save_read++];
				if(save_read>240)
					save_read=0;
			}
			timeout_lidar=0;
			envoiUART.nbChar = nbr_char_to_send+3;
			EnvoiUserUdp(envoiUART);
			ptr_read_buffer_uart_rec = save_write;
		}			

		if((ptr_write_buffer_uart != ptr_read_buffer_uart) && U2STAbits.TRMT != 0)
		{
			// Gestion envoi trame
			U2TXREG = buffer_envoi_uart[ptr_read_buffer_uart++];
			if(ptr_read_buffer_uart >= UART_BUFFER_SIZE)
				ptr_read_buffer_uart=0;
		}

		if(securite_ascenseur)
		{
			messSecuAsc[2] = securite_ascenseur;
			securite_ascenseur=0;
			EnvoiUserUdp(envoiSecuAsc);
		}
	
		if(telem)
		{
			messGP2[2] = telem;
			messGP2[3] = telem_front;
			telem=0;
			EnvoiUserUdp(envoiGP2);
		}

		if(Fin_d_angle_bas == 1)
		{
			Fin_d_angle_bas = 0;
			messbaliserapide[0] = 0xC4;
			messbaliserapide[1] = CMD_REPONSE_CAPTEUR;
			messbaliserapide[2] = ID_CAPTEUR_BALISE_1;

			envoiBaliserapide.nbChar = 3;
			for(i=0;i<2;i++)
			{
				angle = (float)(front_rapide[IDCAPTEUR_BAS][i]) / (float)(periode_tour) * 36000;
				messbaliserapide[envoiBaliserapide.nbChar+i*2] = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				messbaliserapide[envoiBaliserapide.nbChar+1+i*2] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}
			envoiBaliserapide.nbChar = 7;	
			//EnvoiUserUdp(envoiBaliserapide);
		}

		if(Fin_d_angle_haut == 1)
		{
			Fin_d_angle_haut = 0;
			messbaliserapide[0] = 0xC4;
			messbaliserapide[1] = CMD_REPONSE_CAPTEUR;
			messbaliserapide[2] = ID_CAPTEUR_BALISE_2;

			envoiBaliserapide.nbChar = 3;
			for(i=0;i<2;i++)
			{
				angle = (float)(front_rapide[IDCAPTEUR_HAUT][i]) / (float)(periode_tour) * 36000;
				messbaliserapide[envoiBaliserapide.nbChar+i*2] = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				messbaliserapide[envoiBaliserapide.nbChar+1+i*2] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}
			envoiBaliserapide.nbChar = 7;	
			//EnvoiUserUdp(envoiBaliserapide);
		}


		// Gestion Balise
		if(Fin_de_tour == 1)
		{
			Fin_de_tour = 0;
			
			for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
			{
				angle = (float)(buffer_fronts[IDCAPTEUR_HAUT][i]) / (float)(periode_tour) * 36000;
				buffer_angles[IDCAPTEUR_HAUT][2*i]   = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				buffer_angles[IDCAPTEUR_HAUT][2*i+1] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}
			for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
			{
				angle = (float)(buffer_fronts[IDCAPTEUR_BAS][i]) / (float)(periode_tour) * 36000;
				buffer_angles[IDCAPTEUR_BAS][2*i]   = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				buffer_angles[IDCAPTEUR_BAS][2*i+1] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}

			messbalise[0] = 0xC4;
			messbalise[1] = CMD_REPONSE_CAPTEUR;
			messbalise[2] = ID_CAPTEUR_BALISE;
			

			messbalise[3] = (periode_tour >> 8) & 0x00FF;
			messbalise[4] = periode_tour & 0x00FF;
			
			
			messbalise[5] = nombre_angles[IDCAPTEUR_HAUT];
			messbalise[6] = nombre_angles[IDCAPTEUR_BAS];
       		
			envoiBalise.nbChar = 7;
			
       		for(i = 0; i < nombre_angles[IDCAPTEUR_HAUT]; i++)
			{
				messbalise[envoiBalise.nbChar+i*2] = buffer_angles[IDCAPTEUR_HAUT][2*i];	//MSB
       			messbalise[envoiBalise.nbChar+1+i*2] = buffer_angles[IDCAPTEUR_HAUT][2*i+1]; //LSB
			}
			
			envoiBalise.nbChar += nombre_angles[IDCAPTEUR_HAUT]*2;
			
			for(i = 0; i < nombre_angles[IDCAPTEUR_BAS]; i++)
			{
				messbalise[envoiBalise.nbChar+i*2] = buffer_angles[IDCAPTEUR_BAS][2*i];	 //MSB
       			messbalise[envoiBalise.nbChar+1+i*2] = buffer_angles[IDCAPTEUR_BAS][2*i+1]; //LSB
			}
			
			envoiBalise.nbChar += nombre_angles[IDCAPTEUR_BAS]*2;
			
			EnvoiUserUdp(envoiBalise);
			envoiBalise.nbChar = 0;
		}		

		// Gestion Demultiplexage analogique

		if(Old_Coo_BarriereBrasGauche != Coo_BarriereBrasGauche)
		{
			messCapteur_OnOff[2] = ID_BARRIERE_BRAS_GAUCHE;
			messCapteur_OnOff[3] = Coo_BarriereBrasGauche;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_BarriereBrasGauche = Coo_BarriereBrasGauche;
		}
		if(Old_Coo_SwitchBrasGaucheHaut != Coo_SwitchBrasGaucheHaut)
		{
			messCapteur_OnOff[2] = ID_SWITCH_BRAS_GAUCHE_HAUT;
			messCapteur_OnOff[3] = Coo_SwitchBrasGaucheHaut;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_SwitchBrasGaucheHaut = Coo_SwitchBrasGaucheHaut;
		}
		if(Old_Coo_SwitchBrasGaucheBas != Coo_SwitchBrasGaucheBas)
		{
			messCapteur_OnOff[2] = ID_SWITCH_BRAS_GAUCHE_BAS;
			messCapteur_OnOff[3] = Coo_SwitchBrasGaucheBas;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_SwitchBrasGaucheBas = Coo_SwitchBrasGaucheBas;
		}
		if(Old_Coo_SwitchOrigineBrasGauche != Coo_SwitchOrigineBrasGauche)
		{
			messCapteur_OnOff[2] = ID_SWITCH_ORIGINE_BRAS_GAUCHE;
			messCapteur_OnOff[3] = !Coo_SwitchOrigineBrasGauche;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_SwitchOrigineBrasGauche = Coo_SwitchOrigineBrasGauche;
			if(Coo_SwitchOrigineBrasGauche == 0)
			{
				Motors_Stop(ABRUPT,0);
			}
		}
		if(Old_Coo_BarriereBrasDroit != Coo_BarriereBrasDroit)
		{
			messCapteur_OnOff[2] = ID_BARRIERE_BRAS_DROIT;
			messCapteur_OnOff[3] = !Coo_BarriereBrasDroit;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_BarriereBrasDroit = Coo_BarriereBrasDroit;
		}
		if(Old_Coo_SwitchBrasDroitHaut != Coo_SwitchBrasDroitHaut)
		{
			messCapteur_OnOff[2] = ID_SWITCH_BRAS_DROIT_HAUT;
			messCapteur_OnOff[3] = Coo_SwitchBrasDroitHaut;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_SwitchBrasDroitHaut = Coo_SwitchBrasDroitHaut;
		}
		if(Old_Coo_SwitchBrasDroitBas != Coo_SwitchBrasDroitBas)
		{
			messCapteur_OnOff[2] = ID_SWITCH_BRAS_DROIT_BAS;
			messCapteur_OnOff[3] = Coo_SwitchBrasDroitBas;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_SwitchBrasDroitBas = Coo_SwitchBrasDroitBas;
		}
		if(Old_Coo_SwitchOrigineBrasDroit != Coo_SwitchOrigineBrasDroit)
		{
			messCapteur_OnOff[2] = ID_SWITCH_ORIGINE_BRAS_DROIT;
			messCapteur_OnOff[3] = !Coo_SwitchOrigineBrasDroit;
			EnvoiUserUdp(Etat_Capteur_OnOff);	
			Old_Coo_SwitchOrigineBrasDroit = Coo_SwitchOrigineBrasDroit;
			if(Coo_SwitchOrigineBrasDroit == 0)
			{
				Motors_Stop(ABRUPT,1);
			}	
		}

		if(jack_cpt > 20 && jackAvant)
	  	{
		  	EnvoiUserUdp (Jack);
		  	jackAvant = 0;
		}
		if(etatCouleur != MATCH_COULEUR)
		{
			Couleur[2] = MATCH_COULEUR;
  			EnvoiUserUdp (Couleur_Equipe);
  			etatCouleur = MATCH_COULEUR;
  		}
		if(flag_envoi) 
		{	
			scan=0;
			EnvoiUserUdp(envoiFin);
			flag_envoi = 0;
		}
		if(flag_blocage)
		{
			//EnvoiUserUdp(envoiBlocage);
			flag_blocage = 0;
		}
		
		StackTask();
		trame = ReceptionUserUdp();
		if(trame.nbChar != 0)
		{
			trame = AnalyseTrame(trame);
			EnvoiUserUdp(trame);
		}
        StackApplications();

		if(dwLastIP != AppConfig.MyIPAddr.Val)
		{
			dwLastIP = AppConfig.MyIPAddr.Val;
			
			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\nNew IP Address: ");
			#endif

			DisplayIPValue(AppConfig.MyIPAddr);

			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\n");
			#endif


			#if defined(STACK_USE_ANNOUNCE)
				AnnounceIP();
			#endif
		}
	}
}

// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal)
{
//	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
    BYTE IPDigit[4];
	BYTE i;
#ifdef USE_LCD
	BYTE j;
	BYTE LCDPos=16;
#endif

	for(i = 0; i < sizeof(IP_ADDR); i++)
	{
	    uitoa((WORD)IPVal.v[i], IPDigit);

		#if defined(STACK_USE_UART)
			putsUART(IPDigit);
		#endif

		#ifdef USE_LCD
			for(j = 0; j < strlen((char*)IPDigit); j++)
			{
				LCDText[LCDPos++] = IPDigit[j];
			}
			if(i == sizeof(IP_ADDR)-1)
				break;
			LCDText[LCDPos++] = '.';
		#else
			if(i == sizeof(IP_ADDR)-1)
				break;
		#endif

		#if defined(STACK_USE_UART)
			while(BusyUART());
			WriteUART('.');
		#endif
	}

	#ifdef USE_LCD
		if(LCDPos < 32u)
			LCDText[LCDPos] = 0;
		LCDUpdate();
	#endif
}

//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
//	{
//		_prog_addressT MACAddressAddress;
//		MACAddressAddress.next = 0x157F8;
//		_memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
//	}
	AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;


	// SNMP Community String configuration
	#if defined(STACK_USE_SNMP_SERVER)
	{
		BYTE i;
		static ROM char * ROM cReadCommunities[] = SNMP_READ_COMMUNITIES;
		static ROM char * ROM cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
		ROM char * strCommunity;
		
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			// Get a pointer to the next community string
			strCommunity = cReadCommunities[i];
			if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_READ_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.readCommunity[0]))
				while(1);
			
			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.readCommunity[i], strCommunity);

			// Get a pointer to the next community string
			strCommunity = cWriteCommunities[i];
			if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.writeCommunity[0]))
				while(1);

			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.writeCommunity[i], strCommunity);
		}
	}
	#endif

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);

	#if defined(ZG_CS_TRIS)
		// Load the default SSID Name
		if (sizeof(MY_DEFAULT_SSID_NAME) > sizeof(AppConfig.MySSID))
		{
		    ZGSYS_DRIVER_ASSERT(5, (ROM char *)"AppConfig.MySSID[] too small.\n");
		}
		memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
	#endif

	#if defined(EEPROM_CS_TRIS)
	{
		BYTE c;
		
	    // When a record is saved, first byte is written as 0x60 to indicate
	    // that a valid record was saved.  Note that older stack versions
		// used 0x57.  This change has been made to so old EEPROM contents
		// will get overwritten.  The AppConfig() structure has been changed,
		// resulting in parameter misalignment if still using old EEPROM
		// contents.
		XEEReadArray(0x0000, &c, 1);
	    if(c == 0x60u)
		    XEEReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
	    else
	        SaveAppConfig();
	}
	#elif defined(SPIFLASH_CS_TRIS)
	{
		BYTE c;
		
		SPIFlashReadArray(0x0000, &c, 1);
		if(c == 0x60u)
			SPIFlashReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#endif
}

#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
void SaveAppConfig(void)
{
	// Ensure adequate space has been reserved in non-volatile storage to 
	// store the entire AppConfig structure.  If you get stuck in this while(1) 
	// trap, it means you have a design time misconfiguration in TCPIPConfig.h.
	// You must increase MPFS_RESERVE_BLOCK to allocate more space.
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		if(sizeof(AppConfig) > MPFS_RESERVE_BLOCK)
			while(1);
	#endif

	#if defined(EEPROM_CS_TRIS)
	    XEEBeginWrite(0x0000);
	    XEEWrite(0x60);
	    XEEWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #else
	    SPIFlashBeginWrite(0x0000);
	    SPIFlashWrite(0x60);
	    SPIFlashWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #endif
}
#endif

void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void) 
{
	static unsigned int cpt_asser_canon=0;
	static unsigned char k=0;
	unsigned char i=0;
	double temp;
	static double tab_vitesse_canon[16];
	static unsigned int puissance;
	static unsigned char etat_canon=0;	
	static double vitesse_canon_brut;

//	LATBbits.LATB12=1;
	timeout_servo++;
	flag = 0;
	courrier = 1;
	if(MATCH_JACK && 	jack_cpt < 255)	jack_cpt++;
	else								jack_cpt=0;
	
	if(cpt_antirebond_rb3<10)
		cpt_antirebond_rb3++;

	if(timeout_lidar<200) timeout_lidar++;
	
	motor_flag = Motors_Task(); // Si prend trop de ressource sur l'udp, inclure motortask dans le main	
	
	/* TODO : On supprime ?
	if(--desactive_interrupt==0) // Si on passe dedans, c'est qu'on a retrouvé une nouvelle valeur de vitesse
	{
		IFS0bits.IC2IF = 0;
		IEC0bits.IC2IE = 1;
				
		tab_vitesse_canon[k] = (60*1000000/(3.2*(double)capteur_vitesse)); // résultat en tour/min
		vitesse_canon_brut=tab_vitesse_canon[k];
		if(++k>15)	k=0;
		
		temp=0;
		for(i=0;i<16;i++)
		{
			temp+=tab_vitesse_canon[k];
		}
		vitesse_canon = temp/16;
	}

	if(cpt_asser_canon++>prd_asser_canon)
	{
		if(consigne_canon == 0) etat_canon =0;
		else					Canon_Vitesse(puissance);
		switch(etat_canon)
		{
			case 0:	puissance=5000;//IDLE
					if(consigne_canon!=0)	etat_canon++; // WAKE UP !
					break;
			case 1:	prd_asser_canon=300;
					puissance=7200;
					etat_canon++;
					break;
			case 2:	prd_asser_canon=50;
					puissance=5500;
					etat_canon++;
					break;
			case 3:	prd_asser_canon=70;
					if(vitesse_canon_brut<(consigne_canon-15)) puissance+=1;
					if(vitesse_canon_brut>(consigne_canon+15)) puissance-=1;
					break;
		}
		cpt_asser_canon=0;
	}*/
	
	if(motor_flag == 0x10)
	{
		motor_flag=0;
		flag_envoi=1;
	}
	if(motor_flag == 0x30)
	{
		motor_flag=0;
		flag_calage=1;
	}
	if(motor_flag == 0x40)
	{
		motor_flag=0;
		flag_blocage=1;
	}

// Traitement des switchs multiplexes

	Mesure = (double)ADC_Results[5]*COEFF_TENSION_ADC;
	if(Mesure < SeuilsSwitchs[0])		SwitchsStatusGauche = 0;
	else if(Mesure < SeuilsSwitchs[1])	SwitchsStatusGauche = 1;
	else if(Mesure < SeuilsSwitchs[2])	SwitchsStatusGauche = 2;
	else if(Mesure < SeuilsSwitchs[3])	SwitchsStatusGauche = 3;
	else if(Mesure < SeuilsSwitchs[4])	SwitchsStatusGauche = 4;
	else if(Mesure < SeuilsSwitchs[5])	SwitchsStatusGauche = 5;
	else if(Mesure < SeuilsSwitchs[6])	SwitchsStatusGauche = 6;
	else if(Mesure < SeuilsSwitchs[7])	SwitchsStatusGauche = 7;
	else if(Mesure < SeuilsSwitchs[8])	SwitchsStatusGauche = 8;
	else if(Mesure < SeuilsSwitchs[9])	SwitchsStatusGauche = 9;
	else if(Mesure < SeuilsSwitchs[10])	SwitchsStatusGauche = 10;
	else if(Mesure < SeuilsSwitchs[11])	SwitchsStatusGauche = 11;
	else if(Mesure < SeuilsSwitchs[12])	SwitchsStatusGauche = 12;
	else if(Mesure < SeuilsSwitchs[13])	SwitchsStatusGauche = 13;
	else if(Mesure < SeuilsSwitchs[14])	SwitchsStatusGauche = 14;
	else 								SwitchsStatusGauche = 15;

	if(SwitchsStatusGauche != OldSwitchsStatusGauche) 	SwitchsStatusGaucheCpt=0;
	else if(SwitchsStatusGaucheCpt<ANTIREBOND)		SwitchsStatusGaucheCpt++;

	OldSwitchsStatusGauche = SwitchsStatusGauche;	
	
	if(SwitchsStatusGaucheCpt==ANTIREBOND) 
	{
		SwitchsStatusGaucheCpt++;
		switch(SwitchsStatusGauche)
		{
			case 0:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 1:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 2:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 3:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 4:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 5:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 6:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 7:	Coo_BarriereBrasGauche = 0; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 8:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 9:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 10:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 11:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 0;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 12:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 13:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 0; Coo_SwitchBrasGaucheBas = 1;
			break;
			case 14:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 0;
			break;
			case 15:	Coo_BarriereBrasGauche = 1; Coo_SwitchBrasGaucheHaut = 1;  Coo_SwitchOrigineBrasGauche = 1; Coo_SwitchBrasGaucheBas = 1;
			break;
		}
	}

	

	Mesure = (double)ADC_Results[6]*COEFF_TENSION_ADC;

	if(Mesure < SeuilsSwitchs[0])		SwitchsStatusDroit = 0;
	else if(Mesure < SeuilsSwitchs[1])	SwitchsStatusDroit = 1;
	else if(Mesure < SeuilsSwitchs[2])	SwitchsStatusDroit = 2;
	else if(Mesure < SeuilsSwitchs[3])	SwitchsStatusDroit = 3;
	else if(Mesure < SeuilsSwitchs[4])	SwitchsStatusDroit = 4;
	else if(Mesure < SeuilsSwitchs[5])	SwitchsStatusDroit = 5;
	else if(Mesure < SeuilsSwitchs[6])	SwitchsStatusDroit = 6;
	else if(Mesure < SeuilsSwitchs[7])	SwitchsStatusDroit = 7;
	else if(Mesure < SeuilsSwitchs[8])	SwitchsStatusDroit = 8;
	else if(Mesure < SeuilsSwitchs[9])	SwitchsStatusDroit = 9;
	else if(Mesure < SeuilsSwitchs[10])	SwitchsStatusDroit = 10;
	else if(Mesure < SeuilsSwitchs[11])	SwitchsStatusDroit = 11;
	else if(Mesure < SeuilsSwitchs[12])	SwitchsStatusDroit = 12;
	else if(Mesure < SeuilsSwitchs[13])	SwitchsStatusDroit = 13;
	else if(Mesure < SeuilsSwitchs[14])	SwitchsStatusDroit = 14;
	else 																		SwitchsStatusDroit = 15;

	if(SwitchsStatusDroit != OldSwitchsStatusDroit) 	SwitchsStatusDroitCpt=0;
	else if(SwitchsStatusDroitCpt<ANTIREBOND)					SwitchsStatusDroitCpt++;

	OldSwitchsStatusDroit = SwitchsStatusDroit;	
	
	if(SwitchsStatusDroitCpt==ANTIREBOND) 
	{
		SwitchsStatusDroitCpt++;
		switch(SwitchsStatusDroit)
		{
			case 0:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 0;
			break;
			case 1:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 1;
			break;
			case 2:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 0;
			break;
			case 3:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 1;
			break;
			case 4:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 0;
			break;
			case 5:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 1;
			break;
			case 6:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 0;
			break;
			case 7:	Coo_BarriereBrasDroit = 0; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 1;
			break;
			case 8:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 0;
			break;
			case 9:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 1;
			break;
			case 10:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 0;
			break;
			case 11:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 0;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 1;
			break;
			case 12:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 0;
			break;
			case 13:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 0; Coo_SwitchBrasDroitBas = 1;
			break;
			case 14:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 0;
			break;
			case 15:	Coo_BarriereBrasDroit = 1; Coo_SwitchBrasDroitHaut = 1;  Coo_SwitchOrigineBrasDroit = 1; Coo_SwitchBrasDroitBas = 1;
			break;
		}
		
	}
/*
#define ID_SWITCH_BRAS_DROIT_HAUT		0x00
#define ID_SWITCH_BRAS_DROIT_BAS		0x01
#define ID_BARRIERE_BRAS_DROIT			0x02
#define ID_SWITCH_BRAS_GAUCHE_HAUT		0x03
#define ID_SWITCH_BRAS_GAUCHE_BAS		0x04
#define ID_BARRIERE_BRAS_GAUCHE			0x05
#define ID_SWITCH_ORIGINE_BRAS_DROIT	0x06
#define ID_SWITCH_ORIGINE_BRAS_GAUCHE	0x07
*/


	/* TODO : supprimer ?
	Cpt_Tmr2_Capteur_Couleur++;

	if(Cpt_Tmr2_Capteur_Couleur == 20)
	{
		Cpt_Tmr2_Capteur_Couleur = 0;

		switch(++etat_Capteur_Couleur){

			case 1:
				Tab_Capteur_Couleur[0] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 0;
				LED = 0;
			break;
			case 2:
				Tab_Capteur_Couleur[1] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 0;
				LED = 0;
			break;
			case 3:
				Tab_Capteur_Couleur[2] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 1;
				LED = 0;
			break;
			case 4:
				Tab_Capteur_Couleur[3] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 1;
				LED = 0;
			break;
			case 5:
				Tab_Capteur_Couleur[4] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 0;
				LED = 1;
			break;
			case 6:
				Tab_Capteur_Couleur[5] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 0;
				LED = 1;
			break;
			case 7:
				Tab_Capteur_Couleur[6] = Send_Variable_Capteur_Couleur();
				S3  = 0; 
				S2  = 1;
				LED = 1;
			break;
			case 8:
				Tab_Capteur_Couleur[7] = Send_Variable_Capteur_Couleur();
				S3  = 1; 
				S2  = 1;
				LED = 1;
				etat_Capteur_Couleur = 0;
			break;
		}
	}*/
	cpu_status = (TMR4); //Previous value TMR4
//	LATBbits.LATB12=0;
	IFS1bits.T4IF = 0;
}
//Interrupt receive message UART1
//void __attribute__((interrupt,shadow,auto_psv)) _U1RXInterrupt(void) // shadow de merde
void __attribute__((interrupt,auto_psv)) _U1RXInterrupt(void)
{
	static unsigned etat_rx=0;
	static unsigned char recu,recu_ptr;

	IFS0bits.U1RXIF = 0; 		// clear RX interrupt flag
	
	if(U1STAbits.URXDA == 1)
	{
		recu = U1RXREG;
		if(flag_servo==0)
		{
			switch(etat_rx)
			{
				case 0: recu_ptr=0;
						if(recu == 0xFF)	etat_rx++;
						trame_recu[recu_ptr++]=recu;
						break;
				case 1: if(recu == 0xFF)	etat_rx++;
						else	etat_rx=0;
						trame_recu[recu_ptr++]=recu;
						break;
				case 2:	etat_rx++;
						trame_recu[recu_ptr++]=recu;
						break;
				case 3:	recu_nbr = recu;
						etat_rx++;
						trame_recu[recu_ptr++]=recu;
						break;
				case 4:	trame_recu[recu_ptr++]=recu;
						if(recu_ptr == (recu_nbr + 4))
						{
							flag_servo=1;
							etat_rx=0;
						}
						break;
			}
		}
	}			
}


void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void)
{
	unsigned char i;
	TMR2=0;
	IC2CONbits.ICM = 0b011;
	IC7CONbits.ICM = 0b011;
	periode_tour = IC1BUF;
	
	IFS0bits.IC1IF=0;

	nombre_angles[IDCAPTEUR_HAUT] = ptr_fronts_haut;
	nombre_angles[IDCAPTEUR_BAS] = ptr_fronts_bas;

	for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
	{
		buffer_fronts[IDCAPTEUR_HAUT][i] = 	buffer_fronts_temp[IDCAPTEUR_HAUT][i];
	}
	for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
	{
		buffer_fronts[IDCAPTEUR_BAS][i] = 	buffer_fronts_temp[IDCAPTEUR_BAS][i];
	}
	
	ptr_fronts_haut=0;
	ptr_fronts_bas=0;
	Fin_de_tour=1;
	
}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas]=IC2BUF;
	if(ptr_fronts_bas < 8) ptr_fronts_bas++;
	 
	if(ptr_fronts_bas %2 == 0)
	{
		front_rapide[IDCAPTEUR_BAS][1] = buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas-1]; // Front descendant
		Fin_d_angle_bas = 1;
	}
	else
	{
		front_rapide[IDCAPTEUR_BAS][0] = buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas-1]; // Front montant
	}
	IC2CONbits.ICM = 0b001;
	IFS0bits.IC2IF=0;
}

void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut]=IC7BUF;		
	if(ptr_fronts_haut < 8) ptr_fronts_haut++;
	if(ptr_fronts_haut %2 == 0)
	{
		front_rapide[IDCAPTEUR_HAUT][1] = buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut-1]; // Front descendant
		Fin_d_angle_haut = 1;
	}
	else
	{
		front_rapide[IDCAPTEUR_HAUT][0] = buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut-1]; // Front montant
	}
	IC7CONbits.ICM = 0b001;
	IFS1bits.IC7IF=0;
}

//Interrupt receive message UART1
//void __attribute__((interrupt,shadow,auto_psv)) _U1RXInterrupt(void) // shadow de merde
void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void)
{
	static unsigned etat_rx=0;
	static unsigned int recu,recu_ptr=0;

	IFS1bits.U2RXIF = 0; 		// clear RX interrupt flag
	
	if(U2STAbits.URXDA == 1)
	{
		Buffer_passerelle_udpuart[ptr_write_buffer_uart_rec++] = U2RXREG;
		if(ptr_write_buffer_uart_rec>240)
			ptr_write_buffer_uart_rec=0;

		
		
		/*if(flag_servo==0)
		{
			switch(etat_rx)
			{
				case 0: recu_ptr=0;
						if(recu == 0xFF)	etat_rx++;
						trame_recu[recu_ptr++]=recu;
						break;
				case 1: if(recu == 0xFF)	etat_rx++;
						else	etat_rx=0;
						trame_recu[recu_ptr++]=recu;
						break;
				case 2:	etat_rx++;
						trame_recu[recu_ptr++]=recu;
						break;
				case 3:	recu_nbr = recu;
						etat_rx++;
						trame_recu[recu_ptr++]=recu;
						break;
				case 4:	trame_recu[recu_ptr++]=recu;
						if(recu_ptr == (recu_nbr + 4))
						{
							flag_servo=1;
							etat_rx=0;
						}
						break;
			}
		}*/
	}
			
}
