#ifndef _LIBFC_H
#define _LIBFC_H

#define CPU_UNKNOWN 	0
#define CPU_ATMEGA644	1
#define CPU_ATMEGA644P	2
#define CPU_ATMEGA1284	3
#define CPU_ATMEGA1284P	4

extern void LIBFC_Init(unsigned char);
extern void LIBFC_Polling(void);
extern void LIBFC_ReceiverInit(unsigned char rtype);

extern void LIBFC_JetiBox_Putchar(char c);
extern void LIBFC_JetiBox_SetPos(unsigned char index);
extern void LIBFC_JetiBox_Clear(void);
extern void LIBFC_CheckSettings(void);
extern unsigned char LIBFC_GetCPUType(void);

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
extern long ACC_AltitudeFusion(unsigned char init);
#endif

#endif //_LIBFC_H
