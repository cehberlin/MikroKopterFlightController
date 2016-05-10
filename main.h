#ifndef _MAIN_H
#define _MAIN_H

//#define REDUNDANT_FC_SLAVE
//#define REDUNDANT_FC_MASTER
//#define NO_RECEIVER

//#define DEBUG 						// use to activate debug output to MK-Tool: use Debug(text);
//#define ACT_S3D_SUMMENSIGNAL
//#define UserParameter8_FAILSAFE
//#define RECEIVER_SPEKTRUM_DX7EXP
//#define RECEIVER_SPEKTRUM_DX8EXP

// neue Hardware
//#define ROT_OFF   {if((PlatinenVersion == 10)||(PlatinenVersion >= 20)) PORTB &=~0x01; else  PORTB |= 0x01;}
//#define ROT_ON    {if((PlatinenVersion == 10)||(PlatinenVersion >= 20)) PORTB |= 0x01; else  PORTB &=~0x01;}

#define ROT_OFF   {PORTB &=~0x01;}
#define ROT_ON    {PORTB |= 0x01;}

#define ROT_FLASH PORTB ^= 0x01
//#define GRN_OFF   {if((PlatinenVersion < 12) || PlatinenVersion == 25) PORTB &=~0x02; else PORTB |= 0x02;}
//#define GRN_ON    {if((PlatinenVersion < 12) || PlatinenVersion == 25) PORTB |= 0x02; else PORTB &=~0x02;}
#define GRN_OFF   {if(PlatinenVersion == 25) PORTB &=~0x02; else PORTB |= 0x02;}
#define GRN_ON    {if(PlatinenVersion == 25) PORTB |= 0x02; else PORTB &=~0x02;}
#define GRN_FLASH PORTB ^= 0x02

#define SYSCLK F_CPU

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define J3High    PORTD |= 0x20
#define J3Low     PORTD &= ~0x20
#define J4High    PORTD |= 0x10
#define J4Low     PORTD &= ~0x10
#define J5High    PORTD |= 0x08
#define J5Low     PORTD &= ~0x08

extern volatile unsigned char SenderOkay;
extern unsigned char BattLowVoltageWarning;
extern unsigned char BattAutoLandingVoltage, BattComingHomeVoltage;
extern unsigned char CosinusNickWinkel, CosinusRollWinkel;
extern unsigned char PlatinenVersion;
extern unsigned char FoundMotors,DisableRcOffBeeping;
extern unsigned char JetiBeep;
void LipoDetection(unsigned char print);
extern unsigned int FlugMinuten,FlugMinutenGesamt,FlugSekunden;
extern void PrintLine(void);  // "================================="
extern unsigned char ActiveParamSet;
extern unsigned int BL3_Current(unsigned char who); // in 0,1A
extern unsigned char LipoCells;

#include <avr/pgmspace.h>

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))

#endif

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <avr/wdt.h>

#include "old_macros.h"

#include "printf_P.h"
#include "timer0.h"
#include "uart.h"
#include "analog.h"
#include "twimaster.h"
#include "menu.h"
#include "rc.h"
#include "fc.h"
#include "gps.h"
#include "spi.h"
#include "led.h"
#include "Spektrum.h"
#include "capacity.h"
#include "eeprom.h"
#include "libfc.h"
#include "hottmenu.h"
#include "debug.h"
#include "sbus.h"
#include "jeti_ex.h"
#include "M-Link.h"

#endif //_MAIN_H






