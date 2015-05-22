#include <inttypes.h>
#include "main.h"

uint16_t LED1_Timing = 0;
uint16_t LED2_Timing = 0;
unsigned char J16Blinkcount = 0, J16Mask = 1;
unsigned char J17Blinkcount = 0, J17Mask = 1;

// initializes the LED control outputs J16, J17
void LED_Init(void)
{
	// set PC2 & PC3 as output (control of J16 & J17)
	DDRC |= (1<<DDC2)|(1<<DDC3);
	J16_OFF;
	J17_OFF;
	J16Blinkcount = 0; J16Mask = 128;
	J17Blinkcount = 0; J17Mask = 128;
}

// called in UpdateMotors() every 2ms
void LED_Update(void)
{
	static char delay = 0;
	static unsigned char J16Bitmask = 0;
	static unsigned char J17Bitmask = 0;
	static unsigned char J16Warn = 0, J17Warn = 0;
    static unsigned char from_nc = 0;

	if(FromNC_WP_EventChannel != -127) { from_nc = (unsigned char) FromNC_WP_EventChannel + 127; /*beeptime = 300;*/};
	if(!delay--)  // 20ms Intervall
	{
	delay = 9;
	if(FC_StatusFlags & (FC_STATUS_LOWBAT | FC_STATUS_EMERGENCY_LANDING) || (VersionInfo.HardwareError[1] & FC_ERROR1_I2C))
	{
		if(EE_Parameter.WARN_J16_Bitmask)
		 {
          if(!J16Warn) J16Blinkcount = 4;
          J16Warn = 1;
		 }
		if(EE_Parameter.WARN_J17_Bitmask)
		 {
          if(!J17Warn) J17Blinkcount = 4;
          J17Warn = 1;
		 }
	}
	else
	{
        J16Warn = 0;
        J17Warn = 0;
		J16Bitmask = EE_Parameter.J16Bitmask;
		J17Bitmask = EE_Parameter.J17Bitmask;
	}
//DebugOut.Analog[29] = EE_Parameter.GlobalConfig3;
// Output 1
 if(!J16Warn)     
  {
  if((EE_Parameter.BitConfig & CFG_MOTOR_BLINK1) && !MotorenEin) {if(EE_Parameter.BitConfig & CFG_MOTOR_OFF_LED1) J16_ON; else J16_OFF;}
  else
  if((EE_Parameter.J16Timing > 247) && (Parameter_J16Timing > 220)) {if(J16Bitmask & 128) J16_ON; else J16_OFF; J16Mask = 1;}
  else
  if((EE_Parameter.J16Timing > 247) && (Parameter_J16Timing == 5))  {if(J16Bitmask & 128) J16_OFF; else J16_ON; J16Mask = 1;}
  else
  if(!J16Blinkcount--)
   {
    if(EE_Parameter.GlobalConfig3 & CFG3_USE_NC_FOR_OUT1) 
	 {
	  J16Blinkcount = from_nc / 2; 
	  if(!from_nc) { if(J16Bitmask & 128) J16_OFF; else J16_ON; J16Mask = 0; } // Ausschalten
	  else 
	   {
        if(J16Mask == 0) 
		 { 
		  from_nc = 0; 
		  J16Mask = 128; 
		  if(J16Bitmask & 128) J16_OFF; else J16_ON; // Ausschalten
		 } 
		else 
		 {
          if(J16Mask & J16Bitmask) J16_ON; else J16_OFF;
		  J16Mask /= 2;
		 } 
	   }	
	 } 
	else 
	 {
	  J16Blinkcount = Parameter_J16Timing / 2; 
      if(J16Mask == 1) { from_nc = 0; J16Mask = 128; } else J16Mask /= 2;
      if(J16Mask & J16Bitmask) J16_ON; else J16_OFF;
	 } 
   }
  }
  else  // warning case
  if(!J16Blinkcount--)
   {
     J16Blinkcount = 10-1;
     if(J16Mask == 1) J16Mask = 128; else J16Mask /= 2;
     if(J16Mask & EE_Parameter.WARN_J16_Bitmask) J16_ON; else J16_OFF;
   }
// Output 2

 if(!J17Warn)
  {
  if((EE_Parameter.BitConfig & CFG_MOTOR_BLINK2) && !MotorenEin) {if(EE_Parameter.BitConfig & CFG_MOTOR_OFF_LED2) J17_ON; else J17_OFF;}
  else
  if((EE_Parameter.J17Timing > 247) && (Parameter_J17Timing > 220)) {if(J17Bitmask & 128) J17_ON; else J17_OFF; J17Mask = 1;}
  else
  if((EE_Parameter.J17Timing > 247) && (Parameter_J17Timing == 5))  {if(J17Bitmask & 128) J17_OFF; else J17_ON; J17Mask = 1;}
  else
  if(!J17Blinkcount--)
   {
	 J17Blinkcount = Parameter_J17Timing / 2; 
     if(J17Mask == 1) J17Mask = 128; else J17Mask /= 2;
     if(J17Mask & J17Bitmask) J17_ON; else J17_OFF;
   }
  }
  else  // warning case
  if(!J17Blinkcount--)
   {
     J17Blinkcount = 10-1;
     if(J17Mask == 1) J17Mask = 128; else J17Mask /= 2;
     if(J17Mask & EE_Parameter.WARN_J17_Bitmask) J17_ON; else J17_OFF;
   }

   if(PORTC & (1<<PORTC2)) FC_StatusFlags2 |= FC_STATUS2_OUT1_ACTIVE; //else FC_StatusFlags2 &= ~FC_STATUS2_OUT1_ACTIVE;      // Out1 (J16) -> wird in der SPI zurück gesetzt
   if(PORTC & (1<<PORTC3)) FC_StatusFlags2 |= FC_STATUS2_OUT2_ACTIVE; else FC_StatusFlags2 &= ~FC_STATUS2_OUT2_ACTIVE;       // Out2 (J17)
 }
}

