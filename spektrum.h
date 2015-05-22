/*#######################################################################################
Dekodieren eines Spektrum Signals 
#######################################################################################*/

#ifndef _SPEKTRUM_H
#define _SPEKTRUM_H
void SpektrumUartInit(void);
void SpektrumBinding(void);
extern unsigned char SpektrumTimer;
extern void SpektrumParser(unsigned char c);

#endif //_RC_H
