#include <p33FJ128MC804.h>
#include "FonctionsUc.h"

void UART1PutChar( char ch )
{
    U1TXREG = ch;
    while(U1STAbits.TRMT == 0);
}
