#include "main.h"
#include "debug.h"

#ifdef DEBUG															// only include functions if DEBUG is defined in main.h

#warning : "### DEBUG-Funktion aktiv ###"



unsigned char Debug_BufPtr = 0;
struct str_Debug    tDebug;
unsigned char SendDebugOutput = 0;

// function called from _printf_P to output character
void Debug_Putchar(char c)
{
	if (!SendDebugOutput)
	{
	 tDebug.Text[Debug_BufPtr++] = c;									// copy character to buffer
	 if (Debug_BufPtr > 30) Debug_BufPtr = 30;							// avoid buffer overflow
	} 
}

void DebugSend(unsigned char cmd)
{
	if (!SendDebugOutput)
	{
		tDebug.Cmd = cmd;			
		tDebug.Text[Debug_BufPtr] = '\0';						  		// end of text marker
		Debug_BufPtr = 0;												// set bufferindex to 0
		SendDebugOutput = 1;											// set flag to trasmit data the next time in serial transmit function
	}	
}
#endif

/*
add the following code block to the serial transmit function 

#ifdef DEBUG															// only include functions if DEBUG is defined
     if(SendDebugOutput && UebertragungAbgeschlossen)
     {
		 SendOutData('0', FC_ADDRESS, 1, (unsigned char *) &tDebug, sizeof(tDebug));
		 SendDebugOutput = 0;
	 }
#endif	 

*/

