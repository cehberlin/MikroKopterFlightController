#include "libfc.h"
#include "printf_P.h"
#include "main.h"
#include "spi.h"
#include "capacity.h"
#include "jeti_ex.h"
#include "hottmenu.h"

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))

const char PROGMEM JETI_CODE[53] =
{
0, // 0
'O', // SPEAK_ERR_CALIBARTION  1
'P', // SPEAK_ERR_RECEICER 	 2
'Q', // SPEAK_ERR_DATABUS  	 3
'R', //  SPEAK_ERR_NAVI	  	 4
'S', //  SPEAK_ERROR		 5
'T', //  SPEAK_ERR_COMPASS 	 6
'S', //  SPEAK_ERR_SENSOR 	 7
'V', //  SPEAK_ERR_GPS	 	 8
'W', //  SPEAK_ERR_MOTOR	 9
'H', //  SPEAK_MAX_TEMPERAT  10
  0, //  SPEAK_ALTI_REACHED  11
'X', //  SPEAK_WP_REACHED    12
'Y', //  SPEAK_NEXT_WP       13
  0, //  SPEAK_LANDING       14
'Z', //  SPEAK_GPS_FIX       15
'U', //  SPEAK_UNDERVOLTAGE  16
'E', //  SPEAK_GPS_HOLD      17
'F', //  SPEAK_GPS_HOME      18
'G', //  SPEAK_GPS_OFF       19
'H', //  SPEAK_BEEP          20
'A', //  SPEAK_MIKROKOPTER   21
  0, //  SPEAK_CAPACITY      22
'I', //  SPEAK_CF_OFF        23
'B', //  SPEAK_CALIBRATE     24
'J', //  SPEAK_MAX_RANGE     25
'J', //  SPEAK_MAX_ALTITUD   26
  0, // 27
  0, // 28
  0, // 29
  0, // 30
  0, // 31
  0, // 32
  0, // 33
  0, // 34
  0, // 35
  0, // 36
  0, //  SPEAK_20M		  	37   
'D', //  SPEAK_MK_OFF	  	38
'L', //  SPEAK_ALTITUDE_ON 	39
'M', //  SPEAK_ALTITUDE_OFF 40
  0, //  SPEAK_100M		 	41
  0, // 42
  0, // 43
  0, // 44
  0, // 45
'N', //  SPEAK_CF_ON        46
  0, //  SPEAK_SINKING    	47
  0, //  SPEAK_RISING      	48
  0, //  SPEAK_HOLDING     	49
'K', //  SPEAK_GPS_ON      	50 // ?
  0, //  SPEAK_FOLLWING    	51
'C' //  SPEAK_STARTING      52
};


JetiExPacket_t JetiExData[JETI_EX_PARAMETER_COUNT + 1] =    					// Parameter count + DeviceName (ID0) 
{ 
	//	Label[10] 		unit[3], data type,		  Data , position of decimal point
   // 	"1234567890", 	 "123", 
//	{  	"-=.M_K.=-" ,   "   ", 			1, 			0 	, 		0       },    // first one is device name  // datatype 1 = -8192...8192
	{  	"MK        " ,   "   ", 	    1, 			0 	, 		0       },    // first one is device name  // datatype 1 = -8192...8192
	{ 	"Voltage   " ,	 "V  ", 		1, 			0   , 		1       },    // ID 1 
	{  	"Current   " ,	 "A  ", 		1, 			0	, 		1       },    // ID 2
	{  	"Capacity  " ,	 "Ah ", 		1, 			0	, 		2       },    // ID 3		
	{  	"Altitude  " ,	 "m  ", 		1, 			0	, 		0       },    // ID 4		
	{  	"Compass   " ,	 "°  ", 		1, 			0	, 		0       },    // ID 5
	{  	"Sats      " ,	 "   ", 		1, 			0 	, 		0       },    // ID 6
	{  	"Speed     " ,	 "m/s", 		1, 			0 	, 		0       },    // ID 7
	{  	"Distance  " ,	 "m  ", 		1, 			0 	, 		0       },    // ID 8
	{  	"Home-Dir  " ,	 "°  ", 		1, 			0 	, 		0       },    // ID 9
	{  	"max.Temp. " ,	 "°C ", 		1, 			0 	, 		0       },    // ID 10
	{  	"Magn.field" ,	 "%  ", 		1, 			0 	, 		0       },    // ID 11
	{  	"Vario     " ,	 "   ", 		1, 			0 	, 		0       },    // ID 12
	{  	"ErrorCode " ,	 "   ", 		1, 			0 	, 		0       },    // ID 13
	{  	"Latitude  " ,	 "   ", 		9, 			0 	, 		0       },    // ID 14  special data type for coordinates   Import: fixed position in list ID 14 - DO NOT MOVE !!!
	{  	"Longitude " ,	 "   ", 		9, 			0 	, 		0       },    // ID 15  special data type for coordinates   Import: fixed position in list ID 15 - DO NOT MOVE !!!
};


void BuildJeti_Vario(void)
{
 signed int tmp = 0;
 static signed int JetiVarioMeter = 0;
 JetiVarioMeter = (JetiVarioMeter * 3 + VarioMeter) / 4;

 if(VarioCharacter == '+')
  { 
   tmp = (AltitudeSetpointTrimming * EE_Parameter.Hoehe_Verstaerkung) / 32 + 5;
  } 
 else
 if(VarioCharacter == '-')
  { 
   tmp = (AltitudeSetpointTrimming * EE_Parameter.Hoehe_Verstaerkung) / 32 - 5;
  } 
 else
 if((VarioCharacter == ' ') && (FC_StatusFlags & FC_STATUS_FLY))
  {
   tmp = (JetiVarioMeter/32);
  }
 else
 if(VarioCharacter == '^') tmp = FromNC_AltitudeSpeed;
 else 
 if(VarioCharacter == 'v') tmp = tmp - FromNC_AltitudeSpeed;

 JetiExData[12].Value = tmp;
}


// --------------------------------------------------------------------------------------------------
void JetiEX_Update(void)
{

	GetHottestBl();    
 
	JetiExData[1].Value  =  UBat;
	JetiExData[2].Value  =  Capacity.ActualCurrent;
	JetiExData[3].Value  =  Capacity.UsedCapacity / 10;
	JetiExData[4].Value  =  HoehenWert / 100;
	JetiExData[5].Value  =  KompassValue;
	JetiExData[6].Value  =  GPSInfo.NumOfSats;
	JetiExData[7].Value  =  GPSInfo.Speed;
	JetiExData[8].Value  =  GPSInfo.HomeDistance / 10;
	JetiExData[9].Value  =  GPSInfo.HomeBearing;
	JetiExData[10].Value  = MaxBlTemperture;
	JetiExData[11].Value  = EarthMagneticField;
//	JetiExData[12].Value  = Vario; // wird in BuildJeti_Vario() gemacht 
	JetiExData[13].Value  = NC_ErrorCode;
//JetiExData[14].Value =  53 * 0x10000 + 23467; // GPS-Latitude  (macht NC_Fills_HoTT_Telemety() )
//JetiExData[15].Value =   7 * 0x10000 + 23467; // GPS-Longitude (macht NC_Fills_HoTT_Telemety() )
	
}
#endif
