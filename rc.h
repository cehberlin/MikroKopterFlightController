/*#######################################################################################
Derkodieren eines RC Summen Signals 
#######################################################################################*/

#ifndef _RC_H
#define _RC_H

#if defined (__AVR_ATmega644__)
#define TIMER_RELOAD_VALUE  250
#endif

#if defined (__AVR_ATmega644P__)
#define TIMER_RELOAD_VALUE  250
#endif

#define MAX_RC_IN  16+12+3+4 // 16ch + 12ser + 3stages + 4 reserved

extern void rc_sum_init (void);

extern volatile int PPM_in[MAX_RC_IN];
extern volatile int PPM_diff[MAX_RC_IN];  // das diffenzierte Stick-Signal
extern volatile unsigned char NewPpmData;
extern volatile char Channels,tmpChannels;
extern unsigned int PPM_Neutral;
extern signed int ChannelNick,ChannelRoll,ChannelGas,ChannelYaw;

//  0 		-> frei bzw. ACT rssi
//  1 - 16 	-> 1-16
// 17 - 28 	-> 12 Serial channels
// 29 		-> WP-Event kanal
// 30 		-> -127
// 31 		-> 0
// 32 		-> 128

#define SERIAL_POTI_START 17
#define WP_EVENT_PPM_IN   29
#define PPM_IN_OFF        30
#define PPM_IN_MAX        31
#define PPM_IN_MID        32

#define FromNC_WP_EventChannel PPM_in[WP_EVENT_PPM_IN] // WP_EVENT-Channel-Value

#endif //_RC_H
