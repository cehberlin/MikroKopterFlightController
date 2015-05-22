#ifndef _ANALOG_H
 #define _ANALOG_H
/*#######################################################################################

#######################################################################################*/

#define SM_FILTER 16
#define SA_FILTER 512

extern volatile int UBat;
extern volatile int  AdWertNick, AdWertRoll, AdWertGier;
extern volatile int  AdWertAccRoll,AdWertAccNick,AdWertAccHoch;
extern volatile int  HiResNick, HiResRoll;
extern volatile int  AdWertNickFilter, AdWertRollFilter, AdWertGierFilter;
extern volatile int  Aktuell_Nick,Aktuell_Roll,Aktuell_Gier,Aktuell_ax, Aktuell_ay,Aktuell_az;
extern volatile long  Luftdruck;
extern volatile long SummenHoehe;
extern volatile char messanzahl_Druck;
extern volatile unsigned int ZaehlMessungen;
extern unsigned char DruckOffsetSetting;
extern signed char ExpandBaro;
extern volatile int VarioMeter;
extern volatile unsigned int  MessLuftdruck;
extern volatile long StartLuftdruck;
extern volatile char MessanzahlNick;
extern unsigned char AnalogOffsetNick,AnalogOffsetRoll,AnalogOffsetGier;
extern volatile unsigned char AdReady;
volatile long HoehenWertF;

unsigned int ReadADC(unsigned char adc_input);
void         ADC_Init(void);
void SucheLuftruckOffset(void);
void SucheGyroOffset(void);

#define AD_GIER     0
#define AD_ROLL     1
#define AD_NICK     2
#define AD_DRUCK    3
#define AD_UBAT     4
#define AD_ACC_Z    5
#define AD_ACC_Y    6
#define AD_ACC_X    7


#define ANALOG_OFF ADCSRA=0

#define ANALOG_ON ADCSRA=(1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADIE)
 //Signle trigger Mode, Interrupt on
#endif //_ANALOG_H
