//############################################################################
// MULTIPLEX Servo protocol SRXL16 & SRXL12 
//############################################################################

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))

#include "Spektrum.h"
#include "M-Link.h"
#include "main.h"
unsigned char NewMlinkData = 0;
unsigned char MlinkData[36]; 

//############################################################################
// Initializes the UART here
//############################################################################
void MlinkUartInit(void)
{
 SpektrumUartInit(); // same like Spektrum
};

//############################################################################
// Is called by the uart RX interrupt
// UDR contains the received byte
//############################################################################
void MlinkParser(unsigned char udr)
{
 static unsigned char state = 0, lenght = 0; 
 if(!SpektrumTimer)  // Timeout-> block finished
 {
  if(state > 24 && state < 36) // udr = 0xA1 oder A2
   {
	if(udr == 0xA1) lenght = 12 * 2 + 2; // 12 channels plus CRC
	else
	if(udr == 0xA2) lenght = 16 * 2 + 2; // 16 channels plus CRC
	else lenght = 0;
   } else lenght = 0;
   state = 0;
 }
 else
  {
   if(state < sizeof(MlinkData)) 
    {
	 MlinkData[state++] = udr;
	 if(state == lenght)  // last Byte received
	  {
		NewMlinkData = lenght - 2; // without CRC
		lenght = 0;
	  }
	} 
  } 
 SpektrumTimer = 100; // 10ms Timeout
};

void ProcessMlinkData(void)
{
 unsigned char i = 0;
 unsigned int tmp;
 while(i < NewMlinkData)
  {
		 tmp = (unsigned int) MlinkData[i] * 256 + MlinkData[i + 1];
		 tmp /= 13;
		 i += 2;
  	     s_update(i/2, (signed int) tmp - 156); // copies the values into the Channel-Data and calculates the PPM_Diff
		 SenderOkay = 220;  
  }
 Channels = i/2 + 1;
 NewPpmData = 0;  // Null bedeutet: Neue Daten
 NewMlinkData = 0;
}
#endif
