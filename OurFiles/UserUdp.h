/************************************************
*
* UserUdp.h
* Fichier permettant de gérer une communication UDP
* Couche intermédiaire entre le main et le fichier udp.c
*
*
*****************************************************/

#ifndef __USER_UDP__
#define __USER_UDP__

//#include "TCPIP Stack/UDP.h"
#include "TCPIP Stack/TCPIP.h"

typedef struct Trame
{
	BYTE* message;
	int nbChar;
} Trame;

#define UDP_ID 0xC4
#define portReception 12314
#define portEmission 12324

void InitUserUdp();
Trame ReceptionUserUdp();
void EnvoiUserUdp();
void EnvoiStringUdp(const char *string);
void memclr(void * dest, WORD size);

#endif
