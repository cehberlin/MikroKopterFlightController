#include "Spektrum.h"
#include "main.h"
//############################################################################
// Implement your own RC-decoding routines here
//############################################################################


//############################################################################
// Initialize the UART here
//############################################################################
void User_Receiver_Init(void)
{
// SpektrumUartInit(); // or use an existing routine like this
};

//############################################################################
// Is called by the uart RX interrupt
// UDR contains the received byte
//############################################################################
void User_RX_Parser(unsigned char udr)
{
 // place your code here
};

