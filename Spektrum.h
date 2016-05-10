/*#######################################################################################
Dekodieren eines Spektrum Signals 
#######################################################################################*/

#ifndef _SPEKTRUM_H
#define _SPEKTRUM_H
void SpektrumUartInit(void);
void SpektrumBinding(void);
extern unsigned char SpektrumTimer;
extern void SpektrumParser(unsigned char c);
void s_update(unsigned char channel, signed int value);  // Channel-Diff numbercrunching and finally assign new stickvalue to PPM_in

#endif //_RC_H
