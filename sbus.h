#ifndef _SBUS_H
#define _SBUS_H

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
extern unsigned char NewSBusData, sBusBuffer[25];
extern void SbusParser(unsigned char);
extern void SbusUartInit(void);
extern void ProcessSBus(void);

#endif
#endif
