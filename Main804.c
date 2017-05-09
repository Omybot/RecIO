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

//Define Capteur Couleur
#define LED LATAbits.LATA8
#define S2 LATAbits.LATA10
#define S3 LATAbits.LATA7 

// UART
#define UART_BUFFER_SIZE	100
#define SIZE_TRAME_INIT_SERVO 15
unsigned char InitServo[SIZE_TRAME_INIT_SERVO] = {0x04, 0x84, 0x00, 0x08, 0x27, 0x04, 0x84, 0x10, 0x00, 0x19, 0x04, 0x89, 0x10, 0x10, 0x4E};
unsigned char ptr_read_buffer_uart=0,ptr_write_buffer_uart=0;
unsigned char buffer_envoi_uart[UART_BUFFER_SIZE];
unsigned char Buffer_passerelle_udpuart[250];
unsigned char ptr_write_buffer_uart_rec=0;
// Fin UART

// Bits configuration
_FOSCSEL(FNOSC_PRIPLL)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_EC)
_FPOR(FPWRT_PWR1)
_FWDT(FWDTEN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

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

//Variable Capteur Couleur
unsigned int Cpt_Tmr2_Capteur_Couleur = 0;
unsigned int Tab_Capteur_Couleur[8] = {0};
unsigned char etat_Capteur_Couleur = 0;

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
	
	unsigned char i;
	
	Trame trame;		
	
	Trame envoiUART;
	static BYTE messUART[250];
	messUART[0] = 0xC4;
	messUART[1] = CMD_REPONSE_LIDAR;//CMD???;
	messUART[2] = 0xFE;
	envoiUART.message = messUART;
	envoiUART.nbChar = 53;
	
	InitClk(); 		// Initialisation de l'horloge
	InitPorts(); 	// Initialisation des ports E/S
	Init_Timer();	// Initialisation Timer2,Timer4 & Timer5
	InitQEI(); 		// Initialisation des entr�es en quadrature
	InitPWM();		// Configuration du module PWM 
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
	InitUART();	
	Init_Servos();
	Init_Input_Capture();
	InitADC();
	InitDMA();
	DelayMs(100); 
	Go(3000,0);
	Go(1300,1);
	Motors_Power(ON,0);
	Motors_Power(ON,1);

	for(i=0;i<SIZE_TRAME_INIT_SERVO;i++)
	{
		buffer_envoi_uart[ptr_write_buffer_uart++]=InitServo[i];
		if(ptr_write_buffer_uart >= UART_BUFFER_SIZE)
			ptr_write_buffer_uart=0;
	}
	
	while(1)
  	{
		if((ptr_write_buffer_uart != ptr_read_buffer_uart) && U2STAbits.TRMT != 0)
		{
			// Gestion envoi trame
			U2TXREG = buffer_envoi_uart[ptr_read_buffer_uart++];
			if(ptr_read_buffer_uart >= UART_BUFFER_SIZE)
				ptr_read_buffer_uart=0;
		}
	
		StackTask();
		trame = ReceptionUserUdp();
		if(trame.nbChar != 0)
		{
			trame = AnalyseTrame(trame);
			EnvoiUserUdp(trame);
		}
        StackApplications();
	}
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

	flag = 0;
	
	motor_flag = Motors_Task(); // Si prend trop de ressource sur l'udp, inclure motortask dans le main	
	
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
	cpu_status = (TMR4); //Previous value TMR4

	// Gestion capteur de couleur
	Cpt_Tmr2_Capteur_Couleur++;
	
		if(Cpt_Tmr2_Capteur_Couleur == 20)
		{
			Cpt_Tmr2_Capteur_Couleur = 0;
	
			switch(++etat_Capteur_Couleur){
	
				case 1:
//					Tab_Capteur_Couleur[0] = Send_Variable_Capteur_Couleur();
//					S3  = 0; 
//					S2  = 0;
//					LED = 0;
				break;
				case 2:
//					Tab_Capteur_Couleur[1] = Send_Variable_Capteur_Couleur();
//					S3  = 1; 
//					S2  = 0;
//					LED = 0;
				break;
				case 3:
//					Tab_Capteur_Couleur[2] = Send_Variable_Capteur_Couleur();
//					S3  = 0; 
//					S2  = 1;
//					LED = 0;
				break;
				case 4:
//					Tab_Capteur_Couleur[3] = Send_Variable_Capteur_Couleur();
//					S3  = 1; 
//					S2  = 1;
//					LED = 0;
				break;
				case 5:
					S3  = 0; 
					S2  = 0;
					LED = 1;
					Tab_Capteur_Couleur[4] = Send_Variable_Capteur_Couleur();
				break;
				case 6:
//					Tab_Capteur_Couleur[5] = Send_Variable_Capteur_Couleur();
//					S3  = 1; 
//					S2  = 0;
//					LED = 1;
				break;
				case 7:
//					Tab_Capteur_Couleur[6] = Send_Variable_Capteur_Couleur();
//					S3  = 0; 
//					S2  = 1;
//					LED = 1;
				break;
				case 8:
//					Tab_Capteur_Couleur[7] = Send_Variable_Capteur_Couleur();
//					S3  = 1; 
//					S2  = 1;
//					LED = 1;
					etat_Capteur_Couleur = 0;
				break;
			}
	}
	IFS1bits.T4IF = 0;
}
//Interrupt receive message UART1
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

//void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void)
//{
//	TMR2=0;
//	IC2CONbits.ICM = 0b011;
//	IC7CONbits.ICM = 0b011;
//	//IC1BUF;
//	
//	IFS0bits.IC1IF=0;
//}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void)
{
	//IC2BUF;
	IC2CONbits.ICM = 0b001;
	IFS0bits.IC2IF=0;
}

void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt(void)
{
	//IC7BUF;		
	IC7CONbits.ICM = 0b001;
	IFS1bits.IC7IF=0;
}

//Interrupt receive message UART1
void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void)
{
	IFS1bits.U2RXIF = 0; 		// clear RX interrupt flag
	
	if(U2STAbits.URXDA == 1)
	{
		Buffer_passerelle_udpuart[ptr_write_buffer_uart_rec++] = U2RXREG;
		if(ptr_write_buffer_uart_rec>240)
			ptr_write_buffer_uart_rec=0;
	}			
}
