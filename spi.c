// ######################## SPI - FlightCtrl ###################
#include "main.h"
#include "eeprom.h"
#include "uart.h"

//struct str_ToNaviCtrl_Version   ToNaviCtrl_Version;
//struct str_FromNaviCtrl_Version   FromNaviCtrl_Version;
struct str_ToNaviCtrl   ToNaviCtrl;
struct str_FromNaviCtrl   FromNaviCtrl;
struct str_FromNaviCtrl_Value FromNaviCtrl_Value;
struct str_SPI_VersionInfo NC_Version;
struct str_GPSInfo GPSInfo;

unsigned char SPI_BufferIndex;
unsigned char SPI_RxBufferIndex;
signed char FromNC_Rotate_C = 32, FromNC_Rotate_S = 0;
signed char FromNC_WP_EventChannel_New = 0;

volatile unsigned char     SPI_Buffer[sizeof(FromNaviCtrl)];
unsigned char *SPI_TX_Buffer;

unsigned char SPITransferCompleted, SPI_ChkSum;
unsigned char SPI_RxDataValid,NaviDataOkay = 250;

unsigned char SPI_CommandSequence[] = { SPI_FCCMD_STICK, SPI_FCCMD_USER, SPI_FCCMD_SLOW,SPI_FCCMD_BL_ACCU, 
										SPI_FCCMD_STICK, SPI_FCCMD_MISC, SPI_FCCMD_BL_ACCU, 
                                        SPI_FCCMD_STICK, SPI_FCCMD_USER, SPI_FCCMD_BL_ACCU, 
										SPI_FCCMD_STICK, SPI_FCCMD_PARAMETER2, SPI_FCCMD_BL_ACCU
										};
unsigned char SPI_CommandCounter = 0;
unsigned char NC_ErrorCode = 0;
unsigned char NC_GPS_ModeCharacter = ' ';
unsigned char EarthMagneticField = 0;
unsigned char EarthMagneticInclination = 0, EarthMagneticInclinationTheoretic = 0;
unsigned char NC_To_FC_Flags = 0;
unsigned char NCForcesNewDirection = 0; // allows Yawing without CareFree (Yawing at Coming Home)
//unsigned char NC_To_FC_MaxAltitude = 0; // this is a Parameter on the SD-card
signed int POI_KameraNick = 0; // in 0,1°
vector16_t MagVec = {0,0,0};

#ifdef USE_SPI_COMMUNICATION

//------------------------------------------------------
void SPI_MasterInit(void)
{
  DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);    // Set MOSI and SCK output, all others input
  SLAVE_SELECT_DDR_PORT |= (1 << SPI_SLAVE_SELECT);

  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(0<<SPR0)|(0<<SPIE);   // Enable SPI, Master, set clock rate fck/64
  SPSR = 0;//(1<<SPI2X);

  SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);
  SPITransferCompleted = 1;

  //SPDR = 0x00;  // dummy write

  ToNaviCtrl.Sync1 = 0xAA;
  ToNaviCtrl.Sync2 = 0x83;

  ToNaviCtrl.Command = SPI_FCCMD_USER;
  ToNaviCtrl.IntegralNick = 0;
  ToNaviCtrl.IntegralRoll = 0;
  FromNaviCtrl_Value.SerialDataOkay = 0;
  SPI_RxDataValid = 0;

}

//------------------------------------------------------
unsigned char SPI_StartTransmitPacket(void)
{
   if(!SPITransferCompleted) return(0);
   SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
   SPI_TX_Buffer = (unsigned char *) &ToNaviCtrl;

   ToNaviCtrl.Command = SPI_CommandSequence[SPI_CommandCounter++];
   if (SPI_CommandCounter >= sizeof(SPI_CommandSequence)) SPI_CommandCounter = 0;

   SPITransferCompleted = 0;
   UpdateSPI_Buffer();                       // update buffer
   SPI_BufferIndex = 1;
   ToNaviCtrl.Chksum = ToNaviCtrl.Sync1;
   SPDR = ToNaviCtrl.Sync1;                  // Start transmission
   return(1);
}

//------------------------------------------------------
//SIaNAL(SIG_SPI)
void SPI_TransmitByte(void)
{
   static unsigned char SPI_RXState = 0;
   unsigned char rxdata;
   static unsigned char rxchksum;

   if (SPITransferCompleted) return;
   if (!(SPSR & (1 << SPIF))) return;
  BytegapSPI = SPI_BYTEGAP;
//   _delay_us(30);
  SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);   // DeselectSlave

  rxdata = SPDR;
  switch ( SPI_RXState)
  {
  case 0:
			SPI_RxBufferIndex = 0;
			rxchksum = rxdata;
			if (rxdata == 0x81 )  { SPI_RXState  = 1;  }   // 1. Syncbyte ok

   	   break;

   case 1:
 		    if (rxdata == 0x55) { rxchksum += rxdata; SPI_RXState  = 2;  }   // 2. Syncbyte ok
	         else SPI_RXState  = 0;
   	   break;

   case 2:
		   SPI_Buffer[SPI_RxBufferIndex++]= rxdata;             // get data
           //DebugOut.Analog[19]++;
           if (SPI_RxBufferIndex >= sizeof(FromNaviCtrl))
   		   {
      		if (rxdata == rxchksum)
			{
	          unsigned char *ptr = (unsigned char *)&FromNaviCtrl;
   			  memcpy(ptr, (unsigned char *) SPI_Buffer,  sizeof(SPI_Buffer));
			  SPI_RxDataValid = 1;
			}
			else
			 {
			  SPI_RxDataValid = 0;
			 }
			SPI_RXState  = 0;
   		   }
		  else rxchksum += rxdata;
	break;

  }

   if (SPI_BufferIndex < sizeof(ToNaviCtrl))
     {
 	   SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 
	   SPDR = SPI_TX_Buffer[SPI_BufferIndex];
	   ToNaviCtrl.Chksum += SPI_TX_Buffer[SPI_BufferIndex];
	 }
	 else SPITransferCompleted = 1;

	 SPI_BufferIndex++;
}


//------------------------------------------------------
void UpdateSPI_Buffer(void)
{
//  signed int tmp;
  static unsigned char motorindex, oldcommand = SPI_NCCMD_VERSION, slow_command = 0;
  ToNaviCtrl.IntegralNick = (int) (IntegralNick / (long)(EE_Parameter.GyroAccFaktor * 4)); // etwa in 0.1 Grad
  ToNaviCtrl.IntegralRoll = (int) (IntegralRoll / (long)(EE_Parameter.GyroAccFaktor * 4)); // etwa in 0.1 Grad
  ToNaviCtrl.GyroCompass = (10 * ErsatzKompass) / GIER_GRAD_FAKTOR;
  ToNaviCtrl.GyroGier = (signed int) AdNeutralGier - AdWertGier;
  ToNaviCtrl.AccNick = ((int) ACC_AMPLIFY * (NaviAccNick / NaviCntAcc))/4;
  ToNaviCtrl.AccRoll = ((int) ACC_AMPLIFY * (NaviAccRoll / NaviCntAcc))/4;
  NaviCntAcc = 0; NaviAccNick = 0; NaviAccRoll = 0;
//  ToNaviCtrl.User8 = Parameter_UserParam8;
//  ToNaviCtrl.CalState = WinkelOut.CalcState;
   switch(ToNaviCtrl.Command)  //
   {
	 case SPI_FCCMD_USER:
				ToNaviCtrl.Param.Byte[0] = Parameter_UserParam1;
				ToNaviCtrl.Param.Byte[1] = Parameter_UserParam2;
				ToNaviCtrl.Param.Byte[2] = Parameter_UserParam3;
				ToNaviCtrl.Param.Byte[3] = Parameter_UserParam4;
				ToNaviCtrl.Param.Byte[4] = Parameter_UserParam5;
				ToNaviCtrl.Param.Byte[5] = Parameter_UserParam6;
				ToNaviCtrl.Param.Byte[6] = Parameter_UserParam7;
				ToNaviCtrl.Param.Byte[7] = Parameter_UserParam8;
				ToNaviCtrl.Param.Byte[8] = FC_StatusFlags;
				ToNaviCtrl.Param.Byte[9] = FC_StatusFlags2;
//if(FC_StatusFlags2 & FC_STATUS2_WAIT_FOR_TAKEOFF) ToNaviCtrl.Param.Byte[8] &= ~FC_STATUS_FLY;
                FC_StatusFlags &= ~(FC_STATUS_CALIBRATE | FC_STATUS_START);
     			ToNaviCtrl.Param.Int[5] = UBat; // 10 & 11
				if(!(PORTC & (1<<PORTC2))) FC_StatusFlags2 &= ~FC_STATUS2_OUT1_ACTIVE;       // Out1 (J16)
        break;
     case SPI_FCCMD_BL_ACCU:
     			ToNaviCtrl.Param.Int[0] = Capacity.ActualCurrent; // 0.1A
				ToNaviCtrl.Param.Byte[2] = motorindex | Out1ChangedFlag; // 0x80 this Flag marks a changed Out1
				ToNaviCtrl.Param.Byte[3] = Capacity.MinOfMaxPWM;
				ToNaviCtrl.Param.Byte[4] = GetChannelValue(EE_Parameter.NaviGpsModeChannel);  // GPS-Mode control 
				ToNaviCtrl.Param.Byte[5] = VarioCharacter;
				ToNaviCtrl.Param.Byte[6] = Motor[motorindex].NotReadyCnt;
				ToNaviCtrl.Param.Byte[7] = Motor[motorindex].Version;
				ToNaviCtrl.Param.Byte[8] = Motor[motorindex].MaxPWM;
				ToNaviCtrl.Param.Byte[9] = Motor[motorindex].State;
				ToNaviCtrl.Param.Byte[10] = Motor[motorindex].Temperature;
				ToNaviCtrl.Param.Byte[11] = Motor[motorindex].Current;
				if(Mixer.Motor[++motorindex][0] <= 0) // next motor is not used ?
				  while(Mixer.Motor[motorindex][0] <= 0 && motorindex) motorindex = (motorindex + 1) % 13;
				Out1ChangedFlag = 0;
     	break;
	 case SPI_FCCMD_SLOW:
	       switch(slow_command)
           {
			 case 0: 
				ToNaviCtrl.Command = SPI_FCCMD_VERSION; 
				ToNaviCtrl.Param.Byte[0] = VERSION_MAJOR;
				ToNaviCtrl.Param.Byte[1] = VERSION_MINOR;
				ToNaviCtrl.Param.Byte[2] = VERSION_PATCH;
				ToNaviCtrl.Param.Byte[3] = NC_SPI_COMPATIBLE;
				ToNaviCtrl.Param.Byte[4] = PlatinenVersion;
				ToNaviCtrl.Param.Byte[5] = EE_Parameter.LandingSpeed;
				ToNaviCtrl.Param.Byte[6] = EE_Parameter.ComingHomeAltitude;
				ToNaviCtrl.Param.Byte[7] = EE_Parameter.AutoPhotoAtitudes;
				ToNaviCtrl.Param.Byte[8] = VersionInfo.BL_Firmware; 
	            ToNaviCtrl.Param.Byte[9] = ActiveParamSet;
				ToNaviCtrl.Param.Int[5] = FlugMinutenGesamt; // 10 & 11
				slow_command++;  
			 break;	
			 case 1: 
				ToNaviCtrl.Command = SPI_FCCMD_PARAMETER1; 
				ToNaviCtrl.Param.Byte[0] = (unsigned char) BattLowVoltageWarning; //0.1V
				ToNaviCtrl.Param.Byte[1] = EE_Parameter.NaviGpsGain;           // Parameters for the Naviboard
				ToNaviCtrl.Param.Byte[2] = EE_Parameter.NaviGpsP;
				ToNaviCtrl.Param.Byte[3] = EE_Parameter.NaviGpsI;
				ToNaviCtrl.Param.Byte[4] = EE_Parameter.NaviGpsD;
				ToNaviCtrl.Param.Byte[5] = EE_Parameter.NaviGpsA;
				ToNaviCtrl.Param.Byte[6] = EE_Parameter.NaviGpsMinSat;
                ToNaviCtrl.Param.Byte[7] = EE_Parameter.NaviStickThreshold;
                ToNaviCtrl.Param.Byte[8] = EE_Parameter.NaviMaxFlyingRange; 
                ToNaviCtrl.Param.Byte[9] = EE_Parameter.NaviWindCorrection;
                ToNaviCtrl.Param.Byte[10] = EE_Parameter.NaviAccCompensation;
				ToNaviCtrl.Param.Byte[11] = EE_Parameter.NaviAngleLimitation;
				slow_command++;
			 break;	
			 case 2: 
				ToNaviCtrl.Command = SPI_FCCMD_SLOW2; 
				ToNaviCtrl.Param.Int[0] = BoatNeutralNick; // 0 & 1
				ToNaviCtrl.Param.Int[1] = BoatNeutralRoll; // 2 & 3
				ToNaviCtrl.Param.Int[2] = BoatNeutralGier; // 4 & 5
				ToNaviCtrl.Param.Byte[6] = EE_Parameter.CamOrientation;
				ToNaviCtrl.Param.Byte[7] = EE_Parameter.CompassOffset; 
				ToNaviCtrl.Param.Byte[8] = Parameter_GlobalConfig;
				ToNaviCtrl.Param.Byte[9] = Parameter_ExtraConfig;
				ToNaviCtrl.Param.Byte[10] = EE_Parameter.OrientationAngle;
				ToNaviCtrl.Param.Byte[11] = EE_Parameter.GlobalConfig3;
				slow_command++;  
			 break;	
			 case 3: 
				ToNaviCtrl.Command = SPI_FCCMD_SLOW3; 
				ToNaviCtrl.Param.Byte[0] = Parameter_ServoNickControl;
				ToNaviCtrl.Param.Byte[1] = Parameter_ServoRollControl;
				ToNaviCtrl.Param.Byte[2] = EE_Parameter.NaviDescendRange; // in 10m
				ToNaviCtrl.Param.Byte[3] = Parameter_MaximumAltitude;
				ToNaviCtrl.Param.Byte[4] = EE_Parameter.ServoCompInvert;
				ToNaviCtrl.Param.Byte[5] = LipoCells;
				ToNaviCtrl.Param.Int[3] = ShutterCounter; // 6 & 7
				ToNaviCtrl.Param.Byte[8] = LowVoltageLandingActive;
				ToNaviCtrl.Param.Byte[9] = EE_Parameter.FailSafeTime;
ToNaviCtrl.Param.Byte[10] = 0;
ToNaviCtrl.Param.Byte[11] = 0;
				slow_command++;  
 	    	 break;
			 default:
				ToNaviCtrl.Command = SPI_FCCMD_NEUTRAL; 
				ToNaviCtrl.Param.Int[0] = AdNeutralNick; // 0 & 1
				ToNaviCtrl.Param.Int[1] = AdNeutralRoll; // 2 & 3
				ToNaviCtrl.Param.Int[2] = AdNeutralGier; // 4 & 5
				ToNaviCtrl.Param.Byte[6] = EE_Parameter.Driftkomp; 
				ToNaviCtrl.Param.Byte[7] = EE_Parameter.NaviPH_LoginTime;
				ToNaviCtrl.Param.Byte[8] = EE_Parameter.Receiver;
				ToNaviCtrl.Param.Byte[9] = EE_Parameter.NaviGpsPLimit;
				ToNaviCtrl.Param.Byte[10] = EE_Parameter.NaviGpsILimit;
				ToNaviCtrl.Param.Byte[11] = EE_Parameter.NaviGpsDLimit;
				slow_command = 0;
			 break;	
			}	
	    break;
	 case SPI_FCCMD_PARAMETER2:
				ToNaviCtrl.Param.Byte[0] = EE_Parameter.AutoPhotoDistance;     // Distance between Photo releases
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				// create the ToNC_SpeakHoTT
				if(EE_Parameter.Receiver != RECEIVER_HOTT) 
				 {
				  if(JetiBeep != 'B') JetiBeep = pgm_read_byte(&JETI_CODE[HoTT_Waring()]);
				  else HoTT_Waring();
				 } 
				ToNaviCtrl.Param.Byte[1] = ToNC_SpeakHoTT;
#else
				ToNaviCtrl.Param.Byte[1] = 0;
#endif
				ToNaviCtrl.Param.Int[1] = Capacity.UsedCapacity; // mAh    // 2 & 3
				ToNaviCtrl.Param.Byte[4] = LowVoltageHomeActive;
				ToNaviCtrl.Param.Byte[5] = ToNC_Load_WP_List;
				ToNaviCtrl.Param.Byte[6] = ToNC_Load_SingePoint;
				ToNaviCtrl.Param.Byte[7] = ToNC_Store_SingePoint;
				ToNC_Load_WP_List = 0;
				ToNC_Load_SingePoint = 0;
				ToNC_Store_SingePoint = 0;
				if(Parameter_KompassWirkung) ToNaviCtrl.Param.sInt[4] = KompassSollWert; // Pos. 8 & 9
				else ToNaviCtrl.Param.sInt[4] = ErsatzKompassInGrad; // answer with the compass value if the Compass effect is zero
				ToNaviCtrl.Param.Byte[10] = FC_StatusFlags3;
				ToNaviCtrl.Param.Byte[11] = EE_Parameter.SingleWpSpeed;
	    break;
	 case SPI_FCCMD_STICK:
/*
              cli();
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_GAS]];  if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
				ToNaviCtrl.Param.Byte[0] = (char) tmp;
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_GIER]]; if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
				ToNaviCtrl.Param.Byte[1] = (char) tmp;
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]]; if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
				ToNaviCtrl.Param.Byte[2] = (char) tmp;
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_NICK]]; if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
              sei();
				ToNaviCtrl.Param.Byte[3] = (char) tmp;
*/
				ToNaviCtrl.Param.Byte[0] = ChannelGas; 
				ToNaviCtrl.Param.Byte[1] = ChannelYaw; 
				ToNaviCtrl.Param.Byte[2] = ChannelRoll; 
				ToNaviCtrl.Param.Byte[3] = ChannelNick; 
				ToNaviCtrl.Param.Byte[4] = (unsigned char) Poti[0];
				ToNaviCtrl.Param.Byte[5] = (unsigned char) Poti[1];
				ToNaviCtrl.Param.Byte[6] = (unsigned char) Poti[2];
	            ToNaviCtrl.Param.Byte[7] = (unsigned char) Poti[3];
				ToNaviCtrl.Param.Byte[8] = (unsigned char) Poti[4];
				ToNaviCtrl.Param.Byte[9] = (unsigned char) Poti[5];
				ToNaviCtrl.Param.Byte[10] = (unsigned char) Poti[6];
				ToNaviCtrl.Param.Byte[11] = (unsigned char) Poti[7];
			break;
		case SPI_FCCMD_MISC:
			if(WinkelOut.CalcState >= 5)
			{
				WinkelOut.CalcState = 0;
				ToNaviCtrl.Param.Byte[0] = 5;
			}
			else ToNaviCtrl.Param.Byte[0] = WinkelOut.CalcState;
			ToNaviCtrl.Param.Byte[1] = HoverGas / 4;
			ToNaviCtrl.Param.Int[1] = (int)(HoehenWert/5); //2 & 3
			ToNaviCtrl.Param.Int[2] = (int)(SollHoehe/5);  //4 & 5
			ToNaviCtrl.Param.Byte[6] = VersionInfo.HardwareError[0];
			ToNaviCtrl.Param.Byte[7] = VersionInfo.HardwareError[1];
			VersionInfo.HardwareError[0] &= ~FC_ERROR0_CAREFREE; // 			VersionInfo.HardwareError[0] = 0;
			VersionInfo.HardwareError[1] &= FC_ERROR1_MIXER;
			ToNaviCtrl.Param.Byte[8] = DebugOut.Analog[28]; // I2C-Error counter
            ToNaviCtrl.Param.Byte[9] = (unsigned char) SenderOkay;
			ToNaviCtrl.Param.Byte[10] = NC_Wait_for_LED; 
			ToNaviCtrl.Param.Byte[11] = DebugOut.Analog[7] / 4; //GasMischanteil
			break;
	}
      
  if(SPI_RxDataValid)
  {
	if(FromNaviCtrl.Command != oldcommand) NaviDataOkay = 250;
	oldcommand = FromNaviCtrl.Command;
	CalculateCompassTimer = 1;
	if(abs(FromNaviCtrl.GPS_Nick) < 512 && abs(FromNaviCtrl.GPS_Roll) < 512 && (EE_Parameter.GlobalConfig & CFG_GPS_AKTIV))
	{
		GPS_Nick = FromNaviCtrl.GPS_Nick;
		GPS_Roll = FromNaviCtrl.GPS_Roll;
	}

	// update compass readings
//	MagVec.x = FromNaviCtrl.MagVecX;
//	MagVec.y = FromNaviCtrl.MagVecY;
//	MagVec.z = FromNaviCtrl.MagVecZ;

	if(FromNaviCtrl.CompassValue <= 360)   KompassValue = FromNaviCtrl.CompassValue;

    if(FromNaviCtrl.BeepTime > beeptime && !DisableRcOffBeeping) beeptime = FromNaviCtrl.BeepTime;
	  switch (FromNaviCtrl.Command)
	  {
	    case SPI_NCCMD_KALMAN:
			FromNaviCtrl_Value.Kalman_K = FromNaviCtrl.Param.sByte[0];
			FromNaviCtrl_Value.Kalman_MaxFusion = FromNaviCtrl.Param.sByte[1];
			FromNaviCtrl_Value.Kalman_MaxDrift = FromNaviCtrl.Param.sByte[2];
			KompassFusion = FromNaviCtrl.Param.sByte[3];
			if(FromNaviCtrl.Param.Byte[4] & 0x01) NCForcesNewDirection = 1;
			FromNC_Rotate_C = FromNaviCtrl.Param.Byte[5];
			FromNC_Rotate_S = FromNaviCtrl.Param.Byte[6];
			GPS_Aid_StickMultiplikator = FromNaviCtrl.Param.Byte[7];
			if(FromNaviCtrl.Param.sInt[4] >= 0)
			 {
			  NC_CompassSetpoint = FromNaviCtrl.Param.sInt[4]; // bei Carefree kann NC den Kompass-Sollwinkel vorgeben
			 }
			POI_KameraNick = (POI_KameraNick + FromNaviCtrl.Param.sInt[5]) / 2; // FromNaviCtrl.Param.sInt[5]; // Nickwinkel
			break;
		case SPI_NCCMD_VERSION:
			NC_Version.Major = FromNaviCtrl.Param.Byte[0];
			NC_Version.Minor = FromNaviCtrl.Param.Byte[1];
			NC_Version.Patch = FromNaviCtrl.Param.Byte[2];
			NC_Version.Compatible = FromNaviCtrl.Param.Byte[3];
			NC_Version.Hardware = FromNaviCtrl.Param.Byte[4];
			DebugOut.Status[0] |= FromNaviCtrl.Param.Byte[5];
			DebugOut.Status[1] = (DebugOut.Status[1] & (0x01|0x02)) | (FromNaviCtrl.Param.Byte[6] & (0x04 | 0x08));
			NC_ErrorCode = FromNaviCtrl.Param.Byte[7];
			NC_GPS_ModeCharacter = FromNaviCtrl.Param.Byte[8];
			FromNaviCtrl_Value.SerialDataOkay = FromNaviCtrl.Param.Byte[9];
			NC_To_FC_Flags = FromNaviCtrl.Param.Byte[10];
//NC_To_FC_MaxAltitude = FromNaviCtrl.Param.Byte[11];
			break;
		case SPI_NCCMD_GPSINFO:
			GPSInfo.Flags = FromNaviCtrl.Param.Byte[0];
			GPSInfo.NumOfSats = FromNaviCtrl.Param.Byte[1];
			GPSInfo.SatFix = FromNaviCtrl.Param.Byte[2];
			GPSInfo.Speed = FromNaviCtrl.Param.Byte[3];
			GPSInfo.HomeDistance = FromNaviCtrl.Param.Int[2];
			GPSInfo.HomeBearing = FromNaviCtrl.Param.sInt[3];
		    if(!FromNC_WP_EventChannel_New) FromNC_WP_EventChannel_New = (unsigned char) FromNaviCtrl.Param.Byte[8] + 127;  // zwischenspeichern, damit keiner verpasst wird
			PPM_in[WP_EVENT_PPM_IN] = (signed char) FromNaviCtrl.Param.Byte[8]; // WP_EVENT-Channel-Value (FromNC_WP_EventChannel)
 			FromNC_AltitudeSpeed = FromNaviCtrl.Param.Byte[9];
 	        FromNC_AltitudeSetpoint = (long) FromNaviCtrl.Param.sInt[5] * 10;  // in cm
			break;
		case SPI_MISC:
			EarthMagneticField = FromNaviCtrl.Param.Byte[0];
			EarthMagneticInclination = FromNaviCtrl.Param.Byte[1];
			EarthMagneticInclinationTheoretic = FromNaviCtrl.Param.Byte[2];
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			if(FromNaviCtrl.Param.Byte[3]) 
            if(!SpeakHoTT || (SpeakHoTT >= SPEAK_GPS_HOLD && SpeakHoTT <= SPEAK_GPS_OFF)) SpeakHoTT = FromNaviCtrl.Param.Byte[3];
			NaviData_TargetDistance = FromNaviCtrl.Param.Int[3];
#endif
			NaviData_WaypointIndex = FromNaviCtrl.Param.Byte[4];
			NaviData_WaypointNumber = FromNaviCtrl.Param.Byte[5];
			NaviData_TargetHoldTime = FromNaviCtrl.Param.Byte[8];
			NaviData_MaxWpListIndex = FromNaviCtrl.Param.Byte[9];
			CompassCorrected = FromNaviCtrl.Param.sInt[5]; // Bytes 10 & 11
			break;    

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
		case SPI_NCCMD_HOTT_DATA:
			//if(EE_Parameter.Receiver == RECEIVER_HOTT) 
			NC_Fills_HoTT_Telemety();
			break;
#endif

// 0 = 0,1
// 1 = 2,3
// 2 = 4,5
// 3 = 6,7
// 4 = 8,9
// 5 = 10,11
		default:
			break;
	  }
  }
  else
  {
//    KompassValue = 0;
//    KompassRichtung = 0;
	GPS_Nick = 0;
    GPS_Roll = 0;
  }
}

#endif


