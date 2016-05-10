#include <inttypes.h>
#include "main.h"

uint16_t LED1_Timing = 0;
uint16_t LED2_Timing = 0;
unsigned char J16Blinkcount = 0, J16Mask = 1;
unsigned char J17Blinkcount = 0, J17Mask = 1;
unsigned char NC_Wait_for_LED = 0;  // signal to NC: Wait for the LAD PAtter before switching to the next WP
unsigned int ShutterCounter = 0;
unsigned char Out1ChangedFlag = 0; // can be 0 or 0x80
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
	static char delay = 0, last_portC = 0;
	static unsigned char J16Bitmask = 0;
	static unsigned char J17Bitmask = 0;
	static unsigned char J16Warn = 0, J17Warn = 0;
    static unsigned char from_nc = 0; // Copy for the timing

	if(!NC_Wait_for_LED) from_nc = (unsigned char) PPM_in[WP_EVENT_PPM_IN] + 127;

	if(!delay--)  // 20ms Intervall
	{
	J16Bitmask = EE_Parameter.J16Bitmask;
	J17Bitmask = EE_Parameter.J17Bitmask;
	delay = 9;
	if(FC_StatusFlags & (FC_STATUS_LOWBAT | FC_STATUS_EMERGENCY_LANDING) || (VersionInfo.HardwareError[1] & FC_ERROR1_I2C) || !SenderOkay)
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
	}
//DebugOut.Analog[29] = EE_Parameter.GlobalConfig3;
// Output 1
 if(!J16Warn)     
  {
  if((EE_Parameter.BitConfig & CFG_MOTOR_BLINK1) && !MotorenEin) {if(EE_Parameter.BitConfig & CFG_MOTOR_OFF_LED1) J16_ON; else J16_OFF;}
  else
  if((EE_Parameter.J16Timing > 247) && (Parameter_J16Timing > 220)) {if(J16Bitmask & 128) J16_OFF; else J16_ON; J16Mask = 1; NC_Wait_for_LED = 0;}  // Manual overwrite
  else
  if((EE_Parameter.J16Timing > 247) && (Parameter_J16Timing == 5))  {if(J16Bitmask & 128) J16_ON; else J16_OFF; J16Mask = 1; NC_Wait_for_LED = 0;}  // Manual overwrite
  else
  if(!J16Blinkcount--)
   {
    if(EE_Parameter.GlobalConfig3 & CFG3_USE_NC_FOR_OUT1) 
	 {
	  J16Blinkcount = from_nc / 2; 
	  if(!from_nc) { NC_Wait_for_LED = 0; if(J16Bitmask & 128) J16_ON; else J16_OFF; J16Mask = 0; } // Ausschalten
	  else 
	   {
        NC_Wait_for_LED = 1;
        if(J16Mask == 0) 
		 { 
		  from_nc = FromNC_WP_EventChannel_New; 
		  J16Mask = 64; 
		  if(J16Bitmask & 128) J16_ON; else J16_OFF; // Ausschalten
		 } 
		else 
		 {
		  if(J16Mask & J16Bitmask) J16_ON; else J16_OFF;
		  J16Mask /= 2;
		  if(J16Mask == 0x01) FromNC_WP_EventChannel_New = 0;  // Last Bit -> Refresh the value by NC now
		 } 
	   }	
	 } 
	else 
	 {
	  J16Blinkcount = Parameter_J16Timing / 2; 
      if(J16Mask == 1) { from_nc = 0; J16Mask = 64; } else J16Mask /= 2;
      if(J16Mask & J16Bitmask) J16_ON; else J16_OFF;
	  NC_Wait_for_LED = 0;
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
  if((EE_Parameter.J17Timing > 247) && (Parameter_J17Timing > 220)) {if(J17Bitmask & 128) J17_OFF; else J17_ON; J17Mask = 1;}
  else
  if((EE_Parameter.J17Timing > 247) && (Parameter_J17Timing == 5))  {if(J17Bitmask & 128) J17_ON; else J17_OFF; J17Mask = 1;}
  else
  if(!J17Blinkcount--)
   {
	 J17Blinkcount = Parameter_J17Timing / 2; 
     if(J17Mask == 1) J17Mask = 64; else J17Mask /= 2;
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


   if(PORTC & (1<<PORTC2))  // output is on
    {
	 if(!(last_portC & (1<<PORTC2))) 
	  {
	   Out1ChangedFlag = 0x80;// this Flag marks a changed Out1;
	   ShutterCounter++; // count if output swiched to high
	  } 
	 FC_StatusFlags2 |= FC_STATUS2_OUT1_ACTIVE; //else FC_StatusFlags2 &= ~FC_STATUS2_OUT1_ACTIVE;      // Out1 (J16) -> wird in der SPI zurück gesetzt
	} 
   if(PORTC & (1<<PORTC3)) FC_StatusFlags2 |= FC_STATUS2_OUT2_ACTIVE; else FC_StatusFlags2 &= ~FC_STATUS2_OUT2_ACTIVE;       // Out2 (J17)
   last_portC = PORTC;
 }
}

