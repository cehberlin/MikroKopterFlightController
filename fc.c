/*#######################################################################################
Flight Control
#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software Nutzungsbedingungen (english version: see below)
// + der Fa. HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland - nachfolgend Lizenzgeber genannt -
// + Der Lizenzgeber r�umt dem Kunden ein nicht-ausschlie�liches, zeitlich und r�umlich* unbeschr�nktes Recht ein, die im den
// + Mikrocontroller verwendete Firmware f�r die Hardware Flight-Ctrl, Navi-Ctrl, BL-Ctrl, MK3Mag & PC-Programm MikroKopter-Tool 
// + - nachfolgend Software genannt - nur f�r private Zwecke zu nutzen.
// + Der Einsatz dieser Software ist nur auf oder mit Produkten des Lizenzgebers zul�ssig.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die vom Lizenzgeber gelieferte Software ist urheberrechtlich gesch�tzt. Alle Rechte an der Software sowie an sonstigen im
// + Rahmen der Vertragsanbahnung und Vertragsdurchf�hrung �berlassenen Unterlagen stehen im Verh�ltnis der Vertragspartner ausschlie�lich dem Lizenzgeber zu.
// + Die in der Software enthaltenen Copyright-Vermerke, Markenzeichen, andere Rechtsvorbehalte, Seriennummern sowie
// + sonstige der Programmidentifikation dienenden Merkmale d�rfen vom Kunden nicht ver�ndert oder unkenntlich gemacht werden.
// + Der Kunde trifft angemessene Vorkehrungen f�r den sicheren Einsatz der Software. Er wird die Software gr�ndlich auf deren
// + Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Die Haftung des Lizenzgebers wird - soweit gesetzlich zul�ssig - begrenzt in H�he des typischen und vorhersehbaren
// + Schadens. Die gesetzliche Haftung bei Personensch�den und nach dem Produkthaftungsgesetz bleibt unber�hrt. Dem Lizenzgeber steht jedoch der Einwand 
// + des Mitverschuldens offen.
// + Der Kunde trifft angemessene Vorkehrungen f�r den Fall, dass die Software ganz oder teilweise nicht ordnungsgem�� arbeitet.
// + Er wird die Software gr�ndlich auf deren Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Der Kunde wird er seine Daten vor Einsatz der Software nach dem Stand der Technik sichern.
// + Der Kunde ist dar�ber unterrichtet, dass der Lizenzgeber seine Daten im zur Vertragsdurchf�hrung erforderlichen Umfang
// + und auf Grundlage der Datenschutzvorschriften erhebt, speichert, verarbeitet und, sofern notwendig, an Dritte �bermittelt.
// + *) Die r�umliche Nutzung bezieht sich nur auf den Einsatzort, nicht auf die Reichweite der programmierten Software.
// + #### ENDE DER NUTZUNGSBEDINGUNGEN ####'
// +  Hinweis: Informationen �ber erweiterte Nutzungsrechte (wie z.B. Nutzung f�r nicht-private Zwecke) sind auf Anfrage per Email an info(@)hisystems.de verf�gbar.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software LICENSING TERMS
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + of HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland, Germany - the Licensor -
// + The Licensor grants the customer a non-exclusive license to use the microcontroller firmware of the Flight-Ctrl, Navi-Ctrl, BL-Ctrl, and MK3Mag hardware 
// + (the Software) exclusively for private purposes. The License is unrestricted with respect to time and territory*.
// + The Software may only be used with the Licensor's products.
// + The Software provided by the Licensor is protected by copyright. With respect to the relationship between the parties to this
// + agreement, all rights pertaining to the Software and other documents provided during the preparation and execution of this
// + agreement shall be the property of the Licensor.
// + The information contained in the Software copyright notices, trademarks, other legal reservations, serial numbers and other
// + features that can be used to identify the program may not be altered or defaced by the customer.
// + The customer shall be responsible for taking reasonable precautions
// + for the safe use of the Software. The customer shall test the Software thoroughly regarding its suitability for the
// + intended purpose before implementing it for actual operation. The Licensor's liability shall be limited to the extent of typical and
// + foreseeable damage to the extent permitted by law, notwithstanding statutory liability for bodily injury and product
// + liability. However, the Licensor shall be entitled to the defense of contributory negligence.
// + The customer will take adequate precautions in the case, that the software is not working properly. The customer will test
// + the software for his purpose before any operational usage. The customer will backup his data before using the software.
// + The customer understands that the Licensor collects, stores and processes, and, where required, forwards, customer data
// + to third parties to the extent necessary for executing the agreement, subject to applicable data protection and privacy regulations.
// + *) The territory aspect only refers to the place where the Software is used, not its programmed range.
// + #### END OF LICENSING TERMS ####
// + Note: For information on license extensions (e.g. commercial use), please contact us at info(@)hisystems.de.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "main.h"
#include "mymath.h"
#include "isqrt.h"

unsigned char h,m,s;
unsigned int BaroExpandActive = 0;
int MesswertNick,MesswertRoll,MesswertGier,RohMesswertNick,RohMesswertRoll;
int TrimNick, TrimRoll;
int AdNeutralNick = 0,AdNeutralRoll = 0,AdNeutralGier = 0,StartNeutralRoll = 0,StartNeutralNick = 0;
int BoatNeutralNick = 0,BoatNeutralRoll = 0,BoatNeutralGier = 0;
int Mittelwert_AccNick, Mittelwert_AccRoll;
unsigned int NeutralAccX=0, NeutralAccY=0;
int NaviAccNick, NaviAccRoll,NaviCntAcc = 0;
int NeutralAccZ = 0;
signed char NeutralAccZfine = 0;
unsigned char ControlHeading = 0;// in 2�
long IntegralNick = 0;
long IntegralRoll = 0;
long Integral_Gier = 0;
long Mess_IntegralNick = 0;
long Mess_IntegralRoll = 0;
long Mess_Integral_Gier = 0,Mess_Integral_Gier2 = 0;
long MittelIntegralNick,MittelIntegralRoll;
long SummeNick=0,SummeRoll=0;
volatile long Mess_Integral_Hoch = 0;
int  KompassValue = -1;
int  KompassSollWert = 0;
int  NC_CompassSetpoint = -1;
//int  KompassRichtung = 0;
char CalculateCompassTimer = 100;
unsigned char KompassFusion = 32;
unsigned int  KompassSignalSchlecht = 50;
unsigned char  MAX_GAS,MIN_GAS;
unsigned char HoehenReglerAktiv = 0;
unsigned char TrichterFlug = 0;
long Umschlag180Nick = 250000L, Umschlag180Roll = 250000L;
long  ErsatzKompass;
int   ErsatzKompassInGrad, CompassCorrected; // Kompasswert in Grad
int   GierGyroFehler = 0;
char GyroFaktor,GyroFaktorGier;
char IntegralFaktor,IntegralFaktorGier;
int  DiffNick,DiffRoll;
unsigned int StickGasHover = 127;
int HoverGasMin = 0, HoverGasMax = 1023;
int StickNick = 0,StickRoll = 0,StickGier = 0,StickGas = 0;
//int  Poti1 = 0, Poti2 = 0, Poti3 = 0, Poti4 = 0, Poti5 = 0, Poti6 = 0, Poti7 = 0, Poti8 = 0;
unsigned char Poti[9] = {0,0,0,0,0,0,0,0};
volatile unsigned char SenderOkay = 0;
char MotorenEin = 0,StartTrigger = 0;
long HoehenWert = 0;
long SollHoehe = 0;
signed int AltitudeSetpointTrimming = 0;
long FromNC_AltitudeSetpoint = 0;
unsigned char FromNC_AltitudeSpeed = 0;
unsigned char carefree_old = 50; // to make the Beep when switching
signed char WaypointTrimming = 0;
int CompassGierSetpoint = 0;
unsigned char CalibrationDone = 0;
char NeueKompassRichtungMerken = 0;
int LageKorrekturRoll = 0,LageKorrekturNick = 0, HoverGas = 0;
//float Ki =  FAKTOR_I;
int Ki = 10300 / 33;
unsigned char Looping_Nick = 0,Looping_Roll = 0;
unsigned char Looping_Links = 0, Looping_Rechts = 0, Looping_Unten = 0, Looping_Oben = 0;

unsigned char Parameter_Luftdruck_D  = 48;      // Wert : 0-250
unsigned char Parameter_HoehenSchalter = 0;      // Wert : 0-250
unsigned char Parameter_Hoehe_P      = 16;      // Wert : 0-32
unsigned char Parameter_Hoehe_ACC_Wirkung = 58; // Wert : 0-250
unsigned char Parameter_KompassWirkung = 64;    // Wert : 0-250
unsigned char Parameter_Hoehe_TiltCompensation = 100;        // Wert : 0-250
unsigned char Parameter_Gyro_D = 8;             // Wert : 0-250
unsigned char Parameter_Gyro_P = 150;           // Wert : 10-250
unsigned char Parameter_Gyro_I = 150;           // Wert : 0-250
unsigned char Parameter_Gyro_Gier_P = 150;      // Wert : 10-250
unsigned char Parameter_Gyro_Gier_I = 150;      // Wert : 10-250
unsigned char Parameter_Gier_P = 2;             // Wert : 1-20
unsigned char Parameter_I_Faktor = 10;          // Wert : 1-20
unsigned char Parameter_UserParam1 = 0;
unsigned char Parameter_UserParam2 = 0;
unsigned char Parameter_UserParam3 = 0;
unsigned char Parameter_UserParam4 = 0;
unsigned char Parameter_UserParam5 = 0;
unsigned char Parameter_UserParam6 = 0;
unsigned char Parameter_UserParam7 = 0;
unsigned char Parameter_UserParam8 = 0;
unsigned char Parameter_NickControl = 100;
unsigned char Parameter_ServoNickControl = 100;
unsigned char Parameter_ServoRollControl = 100;
unsigned char Parameter_ServoNickComp = 50;
unsigned char Parameter_ServoRollComp = 85;
unsigned char Parameter_LoopGasLimit = 70;
unsigned char Parameter_AchsKopplung1 = 90;
unsigned char Parameter_AchsKopplung2 = 65;
unsigned char Parameter_CouplingYawCorrection = 64;
//unsigned char Parameter_AchsGegenKopplung1 = 0;
unsigned char Parameter_DynamicStability = 100;
unsigned char Parameter_J16Bitmask;             // for the J16 Output
unsigned char Parameter_J16Timing;              // for the J16 Output
unsigned char Parameter_J17Bitmask;             // for the J17 Output
unsigned char Parameter_J17Timing;              // for the J17 Output
unsigned char Parameter_NaviGpsGain;
unsigned char Parameter_NaviGpsP;
unsigned char Parameter_NaviGpsI;
unsigned char Parameter_NaviGpsD;
unsigned char Parameter_NaviGpsA;
//unsigned char Parameter_NaviOperatingRadius;
unsigned char Parameter_NaviWindCorrection;
unsigned char Parameter_NaviSpeedCompensation;
unsigned char Parameter_ExternalControl;
unsigned char Parameter_GlobalConfig;
unsigned char Parameter_ExtraConfig;
unsigned char Parameter_MaximumAltitude;
unsigned char Parameter_Servo3,Parameter_Servo4,Parameter_Servo5;
unsigned char CareFree = 0;
const signed char sintab[31] = { 0, 2, 4, 6, 7, 8, 8, 8, 7, 6, 4, 2, 0, -2, -4, -6, -7, -8, -8, -8, -7, -6, -4, -2, 0, 2, 4, 6, 7, 8, 8}; // 15� steps

int MaxStickNick = 0,MaxStickRoll = 0;
unsigned int  modell_fliegt = 0;
volatile unsigned char FC_StatusFlags = 0, FC_StatusFlags2 = 0;
unsigned char FC_StatusFlags3 = 0;
long GIER_GRAD_FAKTOR = 1291;
signed int KopplungsteilNickRoll,KopplungsteilRollNick;
signed int tmp_motorwert[MAX_MOTORS];
char VarioCharacter = ' ';
unsigned int HooverGasEmergencyPercent = 0; // The gas value for Emergency landing
unsigned int GasIsZeroCnt = 0; // to detect that the gas-stick is down for a while
signed int Variance = 0;
signed int CosAttitude;	// for projection of hoover gas
unsigned char ACC_AltitudeControl = 0;
unsigned char LowVoltageLandingActive = 0;
unsigned char LowVoltageHomeActive = 0;
signed int DriftNick = 0, DriftRoll = 0;
unsigned char ServoFailsafeActive = 0; // moves Servos into the FS-Position

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
#define OPA_OFFSET_STEP 5
#else
#define OPA_OFFSET_STEP 10
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Debugwerte zuordnen
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CopyDebugValues(void)
{
    DebugOut.Analog[0] = ToNaviCtrl.IntegralNick;//IntegralNick / (EE_Parameter.GyroAccFaktor * 4);
    DebugOut.Analog[1] = ToNaviCtrl.IntegralRoll;//IntegralRoll / (EE_Parameter.GyroAccFaktor * 4);
    DebugOut.Analog[2] = Mittelwert_AccNick / 4;
    DebugOut.Analog[3] = Mittelwert_AccRoll / 4;
    DebugOut.Analog[4] = (signed int) AdNeutralGier - AdWertGier;
	DebugOut.Analog[5] = HoehenWert/10;
    DebugOut.Analog[6] = Aktuell_az;//AdWertAccHoch;//(Mess_Integral_Hoch / 512);
    DebugOut.Analog[8] = KompassValue;
    DebugOut.Analog[9] = UBat;
    DebugOut.Analog[10] = SenderOkay;
    DebugOut.Analog[11] = ErsatzKompassInGrad;
    DebugOut.Analog[12] = Motor[0].SetPoint;
    DebugOut.Analog[13] = Motor[1].SetPoint;
    DebugOut.Analog[14] = Motor[2].SetPoint;
    DebugOut.Analog[15] = Motor[3].SetPoint;
    // current pitch, roll, yaw and throttle
    DebugOut.Analog[16] = StickNick;
    DebugOut.Analog[17] = StickRoll;
    DebugOut.Analog[18] = StickGier;
    DebugOut.Analog[19] = StickGas;
    DebugOut.Analog[20] = ServoNickValue;
	DebugOut.Analog[21] = HoverGas; 
    DebugOut.Analog[22] = Capacity.ActualCurrent;
    DebugOut.Analog[23] = Capacity.UsedCapacity;
    DebugOut.Analog[24] = SollHoehe/10;
    DebugOut.Analog[25] = Parameter_ExternalControl > 128; //current external control state
    DebugOut.Analog[26] = CalibrationDone;
    DebugOut.Analog[27] = KompassSollWert;
    DebugOut.Analog[29] = Capacity.MinOfMaxPWM;
    DebugOut.Analog[30] = GPS_Nick;
    DebugOut.Analog[31] = GPS_Roll;
    if(VersionInfo.HardwareError[0] || VersionInfo.HardwareError[1]) DebugOut.Status[1] |= 1; else DebugOut.Status[1] &= 0xfe;
//DebugOut.Analog[16] = Variance;
//DebugOut.Analog[17] = VarioMeter;
//DebugOut.Analog[18] = HoehenWertF;		
//DebugOut.Analog[25] = Parameter_Hoehe_P;
//DebugOut.Analog[26] = Parameter_Luftdruck_D;
}


void Piep(unsigned char Anzahl, unsigned int dauer)
{
 unsigned int wait = 0;
 unsigned char update_spi = 12;
 if(MotorenEin) return; //auf keinen Fall im Flug!
GRN_OFF;
 while(Anzahl--)
 {
  beeptime = dauer;
  wait = dauer;
  while(beeptime || wait) 
   { 
    if(UpdateMotor) 
	 { 
	  UpdateMotor = 0; 
	  if(!beeptime) wait--;
	  LIBFC_Polling();
	  update_spi--;
	 };
	 if(update_spi == 0) // 41Hz
	  { 
	    if(SPI_StartTransmitPacket()) update_spi = 12; 
		else 
		if(BytegapSPI == 0)  SPI_TransmitByte();  
	   }  
	 else if(BytegapSPI == 0)  SPI_TransmitByte(); 
   }
 }
GRN_ON;
}

//############################################################################
// Messwerte beim Ermitteln der Nullage
void CalibrierMittelwert(void)
//############################################################################
{
//    if(PlatinenVersion == 13) SucheGyroOffset();
    // ADC auschalten, damit die Werte sich nicht w�hrend der Berechnung �ndern
	ANALOG_OFF;
	MesswertNick = AdWertNick;
	MesswertRoll = AdWertRoll;
	MesswertGier = AdWertGier;
	Mittelwert_AccNick = ACC_AMPLIFY * AdWertAccNick;
	Mittelwert_AccRoll = ACC_AMPLIFY * AdWertAccRoll;
   // ADC einschalten
    ANALOG_ON;
	Umschlag180Nick = (long) EE_Parameter.WinkelUmschlagNick * 2500L;
	Umschlag180Roll = (long) EE_Parameter.WinkelUmschlagRoll * 2500L;
}

void StoreNeutralToEeprom(void)
{
	BoatNeutralNick = AdNeutralNick;
	BoatNeutralRoll = AdNeutralRoll;
	BoatNeutralGier = AdNeutralGier;
	SetParamWord(PID_ACC_NICK, (uint16_t)NeutralAccX);
	SetParamWord(PID_ACC_ROLL, (uint16_t)NeutralAccY);
	SetParamWord(PID_GYRO_NICK,(uint16_t)BoatNeutralNick);
	SetParamWord(PID_GYRO_ROLL,(uint16_t)BoatNeutralRoll);
	SetParamWord(PID_GYRO_YAW,(uint16_t)BoatNeutralGier);
}

//############################################################################
//  Nullwerte ermitteln
//  Parameter: 0 -> after switch on (ignore ACC-Z fault)
//  Parameter: 1 -> before Start
//  Parameter: 2 -> calibrate and store ACC
//  Parameter: 3 -> use stored Gyro calibration Data from EEPROM (Boat-Mode)
unsigned char SetNeutral(unsigned char AdjustmentMode)  // retuns: "sucess"
//############################################################################
{
	unsigned char i, sucess = 1;
	unsigned int gier_neutral = 0, nick_neutral = 0, roll_neutral = 0, acc_z_neutral = 0, barotest;
    VersionInfo.HardwareError[0] = 0;
//    HEF4017Reset_ON;
	NeutralAccX = 0;
	NeutralAccY = 0;
	NeutralAccZ = 0;
	NeutralAccZfine = 0;

    AdNeutralNick = 0;
	AdNeutralRoll = 0;
	AdNeutralGier = 0;

    Parameter_AchsKopplung1 = 0;
    Parameter_AchsKopplung2 = 0;

    ExpandBaro = 0;
    if(AdjustmentMode == 3)  FC_StatusFlags3 |= FC_STATUS3_BOAT;
	// else FC_StatusFlags3 &= ~FC_STATUS3_BOAT; -> do not clear that

    CalibrierMittelwert();
    Delay_ms_Mess(100);
	CalibrierMittelwert();

    if((EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG))  // H�henregelung aktiviert?
     {
      if((MessLuftdruck > 950) || (MessLuftdruck < 750) || ExpandBaro) SucheLuftruckOffset();
     }
   
	barotest = MessLuftdruck;
#define NEUTRAL_FILTER 32
	OCR0A += OPA_OFFSET_STEP;
	OCR0B = 255 - OCR0A;
    for(i=0; i<NEUTRAL_FILTER; i++)
	 {
	  Delay_ms_Mess(10);
	  gier_neutral += AdWertGier;
	  nick_neutral += AdWertNick;
	  roll_neutral += AdWertRoll;
	  acc_z_neutral += Aktuell_az;
	 }
	if(MessLuftdruck < 1010 && MessLuftdruck > 20) BaroStep = barotest - MessLuftdruck;
	OCR0A -= OPA_OFFSET_STEP;
	OCR0B = 255 - OCR0A;
     AdNeutralNick = (nick_neutral+NEUTRAL_FILTER/2) / (NEUTRAL_FILTER / 8);
	 AdNeutralRoll = (roll_neutral+NEUTRAL_FILTER/2) / (NEUTRAL_FILTER / 8);
	 AdNeutralGier = (gier_neutral+NEUTRAL_FILTER/2) / (NEUTRAL_FILTER);
     NeutralAccZ = (acc_z_neutral+NEUTRAL_FILTER/2) / (NEUTRAL_FILTER);

     StartNeutralRoll = AdNeutralRoll;
     StartNeutralNick = AdNeutralNick;

     if(AdjustmentMode == 2)
     {
	    NeutralAccX = abs(Mittelwert_AccNick) / (2*ACC_AMPLIFY);
	    NeutralAccY = abs(Mittelwert_AccRoll) / (2*ACC_AMPLIFY);
	  	// Save ACC neutral settings to eeprom
		StoreNeutralToEeprom();
    }
    else
    {
		// restore from eeprom
		NeutralAccX = (int16_t)GetParamWord(PID_ACC_NICK);
		NeutralAccY = (int16_t)GetParamWord(PID_ACC_ROLL);
		// strange settings?
		if(((unsigned int) NeutralAccX > 2048) || ((unsigned int) NeutralAccY > 2048)/* || ((unsigned int) NeutralAccZ > 1024)*/)
		{
			printf("\n\rACC not calibrated!\r\n");
			NeutralAccX = abs(Mittelwert_AccNick) / (2*ACC_AMPLIFY);
			NeutralAccY = abs(Mittelwert_AccRoll) / (2*ACC_AMPLIFY);
			sucess = 0;
		}

	// restore from eeprom
	BoatNeutralNick = (int16_t)GetParamWord(PID_GYRO_NICK);
	BoatNeutralRoll = (int16_t)GetParamWord(PID_GYRO_ROLL);
	BoatNeutralGier = (int16_t)GetParamWord(PID_GYRO_YAW);

     if(FC_StatusFlags3 & FC_STATUS3_BOAT) // Read Gyro Data from eeprom
     {
		// strange settings?
		if(((unsigned int) BoatNeutralNick > (600 * 16)) || ((unsigned int) BoatNeutralNick < (400 * 16))
		   || ((unsigned int) BoatNeutralRoll > (600 * 16)) || ((unsigned int) BoatNeutralRoll < (400 * 16))
		   || ((unsigned int) BoatNeutralGier > (600 * 2)) || ((unsigned int) BoatNeutralGier < (400 * 2)))
		{
			printf("\n\rGyro calibration data not valid\r\n");
			sucess = 0;
			FC_StatusFlags3 &= ~FC_STATUS3_BOAT;
		}
		else
		{
			AdNeutralNick = BoatNeutralNick;
			AdNeutralRoll = BoatNeutralRoll;
			AdNeutralGier = BoatNeutralGier;
		}
     }
    }
   	EEAR = EE_DUMMY;  // Set the EEPROM Address pointer to an unused space
	MesswertNick = 0;
    MesswertRoll = 0;
    MesswertGier = 0;
    Delay_ms_Mess(200);
    Mittelwert_AccNick = ACC_AMPLIFY * AdWertAccNick;
    Mittelwert_AccRoll = ACC_AMPLIFY * AdWertAccRoll;
    IntegralNick = EE_Parameter.GyroAccFaktor * (long)Mittelwert_AccNick;
    IntegralRoll = EE_Parameter.GyroAccFaktor * (long)Mittelwert_AccRoll;
    Mess_IntegralNick = IntegralNick;
    Mess_IntegralRoll = IntegralRoll;
    Mess_Integral_Gier = 0;
    KompassSollWert = KompassValue;
	NC_CompassSetpoint = -1;
	KompassSignalSchlecht = 100;
	Umschlag180Nick = ((long) EE_Parameter.WinkelUmschlagNick * 2500L) + 15000L;
	Umschlag180Roll = ((long) EE_Parameter.WinkelUmschlagRoll * 2500L) + 15000L;
    ErsatzKompass = KompassValue * GIER_GRAD_FAKTOR;
    GierGyroFehler = 0;
    LED_Init();
    if(AdjustmentMode != 0) FC_StatusFlags |= FC_STATUS_CALIBRATE;
    FromNaviCtrl_Value.Kalman_K = -1;
    FromNaviCtrl_Value.Kalman_MaxDrift = 0;
    FromNaviCtrl_Value.Kalman_MaxFusion = 32;
    SenderOkay = 100;

    if(ServoActive)	DDRD  |=0x80; // enable J7 -> Servo signal
	else 
	 {
      if((EE_Parameter.ServoCompInvert & SERVO_NICK_INV) && (EE_Parameter.ServoCompInvert & SERVO_RELATIVE)) NickServoValue = 12000;//((128 + 60) * 4 * 16); // neutral position = upper 1/4//	  else	  
	  else NickServoValue = ((128 - 60) * 4 * 16); // neutral position = lower 1/4
	  CalculateServoSignals = 1;
	  CalculateServo(); // nick
	  CalculateServo(); // roll
	 } 

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
    signed int tilt1, tilt2;
		tilt1 = (int)(IntegralNick/GIER_GRAD_FAKTOR);  // nick angle in deg
		tilt2 = (int)(IntegralRoll/GIER_GRAD_FAKTOR);  // roll angle in deg
		tilt1 = (int16_t)ihypot(tilt1,tilt2); 			// tilt angle over all 
		CosAttitude = c_cos_8192(tilt1); 				
		NeutralAccZ = (long)((long) (NeutralAccZ - 512) * 8192 + 4096) / CosAttitude + 512;
		if(tilt1 > 20) sucess = 0; // calibration must be within 20� Tilt angle 
		if(AdjustmentMode != 0 && ACC_AltitudeControl) if((NeutralAccZ < 682 - 30) || (NeutralAccZ > 682 + 35)) { VersionInfo.HardwareError[0] |= FC_ERROR0_ACC_TOP; sucess = 0;};
#else
	NeutralAccZ = (int16_t)GetParamWord(PID_ACC_TOP);
	EEAR = EE_DUMMY;  // Set the EEPROM Address pointer to an unused space
#endif
	if((AdNeutralNick < 150 * 16) || (AdNeutralNick > 850 * 16)) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; };
	if((AdNeutralRoll < 150 * 16) || (AdNeutralRoll > 850 * 16)) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; };
	if((AdNeutralGier < 150 * 2)  || (AdNeutralGier > 850 * 2))  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW; };
	if((NeutralAccX < 300) || (NeutralAccX > 750)) { VersionInfo.HardwareError[0] |= FC_ERROR0_ACC_NICK; };
	if((NeutralAccY < 300) || (NeutralAccY > 750)) { VersionInfo.HardwareError[0] |= FC_ERROR0_ACC_ROLL; };
	if((NeutralAccZ < 512) || (NeutralAccZ > 850)) { VersionInfo.HardwareError[0] |= FC_ERROR0_ACC_TOP; };
    if(VersionInfo.HardwareError[0]) sucess = 0;
    carefree_old = 70;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	LIBFC_HoTT_Clear();
	ACC_AltitudeFusion(2); // initalisation
#endif
    StartLuftdruck = Luftdruck;
    VarioMeter = 0;
	SummenHoehe = 0;    Mess_Integral_Hoch = 0;
	DebugOut.Analog[28] = 0; // I2C-Counter
	CalcExpandBaroStep();
	if(FC_StatusFlags3 & FC_STATUS3_BOAT && !EE_Parameter.Driftkomp) EE_Parameter.Driftkomp = 4;
	ShutterCounter = 0;
/*
//+++++++++++++++++++++++++++++++++++++++++++
//For testing the expandBaro at 30m				
				ExpandBaro -= 1;
				OCR0A = DruckOffsetSetting - OPA_OFFSET_STEP * ExpandBaro; // increase offset to shift ADC down
				OCR0B = 255 - OCR0A;
				CalcExpandBaroStep();
//+++++++++++++++++++++++++++++++++++++++++++
*/
 return(sucess);
}


//############################################################################
// Bearbeitet die Messwerte
void Mittelwert(void)
//############################################################################
{
    static signed long tmpl,tmpl2,tmpl3,tmpl4;
	static signed int oldNick, oldRoll, d2Roll, d2Nick;
	signed long winkel_nick, winkel_roll;
	MesswertGier = (signed int) AdNeutralGier - AdWertGier;
    MesswertNick = (signed int) AdWertNickFilter / 8;
    MesswertRoll = (signed int) AdWertRollFilter / 8;
    RohMesswertNick = MesswertNick;
    RohMesswertRoll = MesswertRoll;

// Beschleunigungssensor  ++++++++++++++++++++++++++++++++++++++++++++++++
	Mittelwert_AccNick = (Mittelwert_AccNick * 3 + ((ACC_AMPLIFY * AdWertAccNick))) / 4L;
	Mittelwert_AccRoll = (Mittelwert_AccRoll * 3 + ((ACC_AMPLIFY * AdWertAccRoll))) / 4L;
    NaviAccNick    += AdWertAccNick;
    NaviAccRoll    += AdWertAccRoll;
    NaviCntAcc++;

//++++++++++++++++++++++++++++++++++++++++++++++++
	HoehenWert = HoehenWert_Mess;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	HoehenWertF = HoehenWertF_Mess;
#else
	HoehenWertF = HoehenWert;
#endif
//++++++++++++++++++++++++++++++++++++++++++++++++
// ADC einschalten
    ANALOG_ON;
	AdReady = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++

    if(Mess_IntegralRoll > 93000L) winkel_roll = 93000L;
	else if(Mess_IntegralRoll <-93000L) winkel_roll = -93000L;
	else winkel_roll = Mess_IntegralRoll;

    if(Mess_IntegralNick > 93000L) winkel_nick = 93000L;
	else if(Mess_IntegralNick <-93000L) winkel_nick = -93000L;
	else winkel_nick = Mess_IntegralNick;

// Gier  ++++++++++++++++++++++++++++++++++++++++++++++++
   Mess_Integral_Gier += MesswertGier;
   ErsatzKompass += MesswertGier;
// Kopplungsanteil  +++++++++++++++++++++++++++++++++++++
      if(!Looping_Nick && !Looping_Roll)// && (Parameter_GlobalConfig & CFG_ACHSENKOPPLUNG_AKTIV))
         {
            tmpl3 = (MesswertRoll * winkel_nick) / 2048L;
            tmpl3 *= Parameter_AchsKopplung2; //65
            tmpl3 /= 4096L;
            tmpl4 = (MesswertNick * winkel_roll) / 2048L;
            tmpl4 *= Parameter_AchsKopplung2; //65
            tmpl4 /= 4096L;
            KopplungsteilNickRoll = tmpl3;
            KopplungsteilRollNick = tmpl4;
            tmpl4 -= tmpl3;
            ErsatzKompass += tmpl4;
            if(!Parameter_CouplingYawCorrection) Mess_Integral_Gier -= tmpl4/2; // Gier nachhelfen

            tmpl = ((MesswertGier + tmpl4) * winkel_nick) / 2048L;
            tmpl *= Parameter_AchsKopplung1;  // 90
            tmpl /= 4096L;
            tmpl2 = ((MesswertGier + tmpl4) * winkel_roll) / 2048L;
            tmpl2 *= Parameter_AchsKopplung1;
            tmpl2 /= 4096L;
            if(abs(MesswertGier) > 64) if(labs(tmpl) > 128 || labs(tmpl2) > 128) TrichterFlug = 1;
            //MesswertGier += (Parameter_CouplingYawCorrection * tmpl4) / 256;
         }
      else  tmpl = tmpl2 = KopplungsteilNickRoll = KopplungsteilRollNick = 0;
			TrimRoll = tmpl - tmpl2 / 100L;
			TrimNick = -tmpl2 + tmpl / 100L;
// Kompasswert begrenzen  ++++++++++++++++++++++++++++++++++++++++++++++++
 		    if(ErsatzKompass >= (360L * GIER_GRAD_FAKTOR)) ErsatzKompass -= 360L * GIER_GRAD_FAKTOR;  // 360� Umschlag
 		    if(ErsatzKompass < 0)                          ErsatzKompass += 360L * GIER_GRAD_FAKTOR;
// Roll  ++++++++++++++++++++++++++++++++++++++++++++++++
            Mess_IntegralRoll +=  MesswertRoll + TrimRoll - LageKorrekturRoll;
            if(Mess_IntegralRoll > Umschlag180Roll)
            {
             Mess_IntegralRoll  = -(Umschlag180Roll - 25000L);
            }
            if(Mess_IntegralRoll <-Umschlag180Roll)
            {
             Mess_IntegralRoll =  (Umschlag180Roll - 25000L);
            }
// Nick  ++++++++++++++++++++++++++++++++++++++++++++++++
            Mess_IntegralNick  += MesswertNick + TrimNick - LageKorrekturNick;
            if(Mess_IntegralNick > Umschlag180Nick)
             {
              Mess_IntegralNick = -(Umschlag180Nick - 25000L);
             }
            if(Mess_IntegralNick <-Umschlag180Nick)
            {
             Mess_IntegralNick =  (Umschlag180Nick - 25000L);
            }

    Integral_Gier  = Mess_Integral_Gier;
    IntegralNick = Mess_IntegralNick;
    IntegralRoll = Mess_IntegralRoll;


#define D_LIMIT 128

   MesswertNick = HiResNick / 8;
   MesswertRoll = HiResRoll / 8;

   if(AdWertNick < 15)   MesswertNick = -1000;  if(AdWertNick <  7)   MesswertNick = -2000;
//   if(PlatinenVersion == 10)  { if(AdWertNick > 1010) MesswertNick = +1000;  if(AdWertNick > 1017) MesswertNick = +2000; }
//   else  
   {  if(AdWertNick > 2000) MesswertNick = +1000;  if(AdWertNick > 2015) MesswertNick = +2000; }
   if(AdWertRoll < 15)   MesswertRoll = -1000;  if(AdWertRoll <  7)   MesswertRoll = -2000;
//   if(PlatinenVersion == 10) { if(AdWertRoll > 1010) MesswertRoll = +1000;  if(AdWertRoll > 1017) MesswertRoll = +2000; }
//   else 
   { if(AdWertRoll > 2000) MesswertRoll = +1000;  if(AdWertRoll > 2015) MesswertRoll = +2000;  }

  if(Parameter_Gyro_D)
  {
   d2Nick = HiResNick - oldNick;
   oldNick = (oldNick + HiResNick)/2;
   if(d2Nick > D_LIMIT) d2Nick = D_LIMIT;
   else if(d2Nick < -D_LIMIT) d2Nick = -D_LIMIT;

   d2Roll = HiResRoll - oldRoll;
   oldRoll = (oldRoll + HiResRoll)/2;
   if(d2Roll > D_LIMIT) d2Roll = D_LIMIT;
   else if(d2Roll < -D_LIMIT) d2Roll = -D_LIMIT;

   MesswertNick += (d2Nick * (signed int) Parameter_Gyro_D) / 16;
   MesswertRoll += (d2Roll * (signed int) Parameter_Gyro_D) / 16;
   HiResNick += (d2Nick * (signed int) Parameter_Gyro_D);
   HiResRoll += (d2Roll * (signed int) Parameter_Gyro_D);
  }

 if(RohMesswertRoll > 0) TrimRoll  += ((long) abs(KopplungsteilNickRoll) * Parameter_CouplingYawCorrection) / 64L;
 else                    TrimRoll -= ((long) abs(KopplungsteilNickRoll) * Parameter_CouplingYawCorrection) / 64L;
 if(RohMesswertNick > 0) TrimNick += ((long) abs(KopplungsteilRollNick) * Parameter_CouplingYawCorrection) / 64L;
 else                    TrimNick -= ((long) abs(KopplungsteilRollNick) * Parameter_CouplingYawCorrection) / 64L;

  if(Parameter_GlobalConfig & CFG_DREHRATEN_BEGRENZER && !Looping_Nick && !Looping_Roll)
  {
    if(RohMesswertNick > 256)       MesswertNick += 1 * (RohMesswertNick - 256);
    else if(RohMesswertNick < -256) MesswertNick += 1 * (RohMesswertNick + 256);
    if(RohMesswertRoll > 256)       MesswertRoll += 1 * (RohMesswertRoll - 256);
    else if(RohMesswertRoll < -256) MesswertRoll += 1 * (RohMesswertRoll + 256);
  }
}

//############################################################################
// Senden der Motorwerte per I2C-Bus
void SendMotorData(void)
//############################################################################
{
 unsigned char i;
    if(!MotorenEin)
        {
         FC_StatusFlags &= ~(FC_STATUS_MOTOR_RUN | FC_STATUS_FLY);
		 FC_StatusFlags2 &= ~FC_STATUS2_WAIT_FOR_TAKEOFF;
		 for(i=0;i<MAX_MOTORS;i++)
		  {
		   if(!PC_MotortestActive)  MotorTest[i] = 0;
 		   Motor[i].SetPoint = MotorTest[i];
		   Motor[i].SetPointLowerBits = 0;
/*
 Motor[i].SetPoint = MotorTest[i] / 4;            // testing the high resolution
 Motor[i].SetPointLowerBits = MotorTest[i] % 4;
*/
		  }
          if(PC_MotortestActive) PC_MotortestActive--;
        }
	else FC_StatusFlags |= FC_STATUS_MOTOR_RUN;

// ++++++++++++++++++++++++++++++++++++++++++
 
// ++++++++++++++++++++++++++++++++++++++++++
#ifdef REDUNDANT_FC_SLAVE
	I2CTimeout = 100;
	return; // don't send I2C-Data
#endif

#ifdef REDUNDANT_FC_MASTER
if(Parameter_UserParam7 > 150)
 {
	FC_StatusFlags3 |= FC_STATUS3_REDUNDANCE_TEST; 
	// Make noise
	if((BeepMuster == 0xffff)) { beeptime = 25000; BeepMuster = 0x0080; }
	// Do not send I2C-Data
	if((FC_StatusFlags3 & FC_STATUS3_REDUNDANCE_AKTIVE) || !(FC_StatusFlags |= FC_STATUS_MOTOR_RUN)) 
	 {
	  I2CTimeout = 100;
if(modell_fliegt != 0 && modell_fliegt < 100) modell_fliegt = 100; // ansonsten kann es vorkommen, dass die Master-FC auf Standgas bleibt
	  return; // don't send I2C-Data
	 } 
 }
#endif
    if(I2C_TransferActive)
	 {
	  I2C_TransferActive = 0; // enable for the next time
	 }
	else
    {
//     motor_write = 0;
     I2C_Start(TWI_STATE_MOTOR_TX); //Start I2C Interrupt Mode
	}
}

unsigned char GetChannelValue(unsigned char ch) // gives the unsigned value of the channel
{
 int tmp2;
 if(ch == 0) return(0);
 tmp2 = PPM_in[ch] + 127;
 if(tmp2 > 255) tmp2 = 255; else if(tmp2 < 0) tmp2 = 0;
 return(tmp2);	 
}

//############################################################################
// Tr�gt ggf. das Poti als Parameter ein
void ParameterZuordnung(void)
//############################################################################
{
 unsigned char tmp,i;
  for(i=0;i<8;i++)
    {
     int tmp2;
	 tmp = EE_Parameter.Kanalbelegung[K_POTI1 + i];
	 tmp2 = PPM_in[tmp] + 127;
	 if(tmp2 > 255) tmp2 = 255; else if(tmp2 < 0) tmp2 = 0;

	 if(tmp == 25) Poti[i] = tmp2; // 25 = WaypointEvent channel -> no filter
	 else 
     if(tmp2 != Poti[i])
	  {
	   Poti[i] += (tmp2 - Poti[i]) / 4;
       if(Poti[i] > tmp2) Poti[i]--;
	   else Poti[i]++;
	  }
	}
 CHK_POTI_MM(Parameter_Luftdruck_D,EE_Parameter.Luftdruck_D,0,100);
 CHK_POTI_MM(Parameter_Hoehe_P,EE_Parameter.Hoehe_P,0,100);
 CHK_POTI_MM(Parameter_Gyro_P,EE_Parameter.Gyro_P,10,255);
 CHK_POTI_MM(Parameter_J16Timing,EE_Parameter.J16Timing,5,255);
 CHK_POTI_MM(Parameter_J17Timing,EE_Parameter.J17Timing,5,255);
 if(EE_Parameter.Servo3 == 247) { if(PORTC & (1<<PORTC2)) Parameter_Servo3 = EE_Parameter.Servo3OnValue; else Parameter_Servo3 = EE_Parameter.Servo3OffValue;}       // Out1 (J16)
 else if(EE_Parameter.Servo3 == 246) { if(PORTC & (1<<PORTC3)) Parameter_Servo3 = EE_Parameter.Servo3OnValue; else Parameter_Servo3 = EE_Parameter.Servo3OffValue;}
 else CHK_POTI_MM(Parameter_Servo3,EE_Parameter.Servo3, 24, 255);

 if(EE_Parameter.Servo4 == 247) { if(PORTC & (1<<PORTC2)) Parameter_Servo4 = EE_Parameter.Servo4OnValue; else Parameter_Servo4 = EE_Parameter.Servo4OffValue;}
 else if(EE_Parameter.Servo4 == 246) { if(PORTC & (1<<PORTC3)) Parameter_Servo4 = EE_Parameter.Servo4OnValue; else Parameter_Servo4 = EE_Parameter.Servo4OffValue;}  // Out2 (J17)
 else CHK_POTI_MM(Parameter_Servo4,EE_Parameter.Servo4, 24, 255);

 CHK_POTI_MM(Parameter_Servo5,EE_Parameter.Servo5, 24, 255);
 Parameter_HoehenSchalter = GetChannelValue(EE_Parameter.HoeheChannel);
 CHK_POTI(Parameter_Hoehe_ACC_Wirkung,EE_Parameter.Hoehe_ACC_Wirkung);
 CHK_POTI(Parameter_Hoehe_TiltCompensation,EE_Parameter.Hoehe_TiltCompensation);
 CHK_POTI(Parameter_KompassWirkung,EE_Parameter.KompassWirkung);
 CHK_POTI(Parameter_Gyro_I,EE_Parameter.Gyro_I);
 CHK_POTI(Parameter_Gyro_D,EE_Parameter.Gyro_D);
 CHK_POTI(Parameter_Gyro_Gier_P,EE_Parameter.Gyro_Gier_P);
 CHK_POTI(Parameter_Gyro_Gier_I,EE_Parameter.Gyro_Gier_I);
 CHK_POTI(Parameter_I_Faktor,EE_Parameter.I_Faktor);
 CHK_POTI(Parameter_UserParam1,EE_Parameter.UserParam1);
 CHK_POTI(Parameter_UserParam2,EE_Parameter.UserParam2);
 CHK_POTI(Parameter_UserParam3,EE_Parameter.UserParam3);
 CHK_POTI(Parameter_UserParam4,EE_Parameter.UserParam4);
 CHK_POTI(Parameter_UserParam5,EE_Parameter.UserParam5);
 CHK_POTI(Parameter_UserParam6,EE_Parameter.UserParam6);
 CHK_POTI(Parameter_UserParam7,EE_Parameter.UserParam7);
 CHK_POTI(Parameter_UserParam8,EE_Parameter.UserParam8);
 CHK_POTI(Parameter_ServoNickControl,EE_Parameter.ServoNickControl);
 CHK_POTI(Parameter_ServoRollControl,EE_Parameter.ServoRollControl);
 CHK_POTI(Parameter_ServoNickComp,EE_Parameter.ServoNickComp);
 CHK_POTI(Parameter_ServoRollComp,EE_Parameter.ServoRollComp);
 CHK_POTI(Parameter_LoopGasLimit,EE_Parameter.LoopGasLimit);
 CHK_POTI(Parameter_AchsKopplung1,EE_Parameter.AchsKopplung1);
 CHK_POTI(Parameter_AchsKopplung2,EE_Parameter.AchsKopplung2);
 CHK_POTI(Parameter_CouplingYawCorrection,EE_Parameter.CouplingYawCorrection);
 CHK_POTI(Parameter_MaximumAltitude,EE_Parameter.MaxAltitude);
// if((NC_To_FC_MaxAltitude && NC_To_FC_MaxAltitude < Parameter_MaximumAltitude) || Parameter_MaximumAltitude == 0) Parameter_MaximumAltitude = NC_To_FC_MaxAltitude;
 Parameter_GlobalConfig = EE_Parameter.GlobalConfig;
 Parameter_ExtraConfig = EE_Parameter.ExtraConfig;
// CHK_POTI(Parameter_AchsGegenKopplung1,EE_Parameter.AchsGegenKopplung1,0,255);
 CHK_POTI(Parameter_DynamicStability,EE_Parameter.DynamicStability);
 CHK_POTI(Parameter_ExternalControl,EE_Parameter.ExternalControl);
 Ki = 10300 / (Parameter_I_Faktor + 1);
 MAX_GAS = EE_Parameter.Gas_Max;
 MIN_GAS = EE_Parameter.Gas_Min;

 if(EE_Parameter.CareFreeChannel)
   {
	CareFree = 1;
	if(PPM_in[EE_Parameter.CareFreeChannel] < -64) CareFree = 0; 
//    if(tmp >= 248 && Poti[255 - tmp] < 50) CareFree = 0; 
    if(carefree_old != CareFree) 
    {
      if(carefree_old < 3)
	   {
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
		if(CareFree) { beeptime = 1500;  if(!SpeakHoTT) SpeakHoTT = SPEAK_CF_ON; }
	    else { beeptime = 200;  if(!SpeakHoTT) SpeakHoTT = SPEAK_CF_OFF; }
#else
		if(CareFree) beeptime = 1500; 
	    else beeptime = 200;
#endif
		NeueKompassRichtungMerken = 5;
        carefree_old = CareFree;
	   } else carefree_old--;
	}  
#ifndef REDUNDANT_FC_SLAVE
	if(FromNaviCtrl.CompassValue < 0 && CareFree) VersionInfo.HardwareError[0] |= FC_ERROR0_CAREFREE; //else VersionInfo.HardwareError[0] &= ~FC_ERROR0_CAREFREE;
#endif
   }
   else 
   {
    CareFree = 0;
	carefree_old = 10;
   }	

 if(FromNaviCtrl.CompassValue < 0 && MotorenEin && CareFree) // ung�ltiger Kompasswert
	{
	 if(BeepMuster == 0xffff) { beeptime = 15000; BeepMuster = 0xA400;};
	 CareFree = 0;
    }
 if(CareFree) { FC_StatusFlags2 |= FC_STATUS2_CAREFREE; } else FC_StatusFlags2 &= ~FC_STATUS2_CAREFREE;
}

void CalcStickGasHover(void)
{
 if(!EE_Parameter.Hoehe_StickNeutralPoint)
	{
 	  StickGasHover = HoverGas/STICK_GAIN; // rescale back to stick value
  	  StickGasHover = (unsigned int)((unsigned int) StickGasHover * UBat) / BattLowVoltageWarning;
	}
  else StickGasHover = EE_Parameter.Hoehe_StickNeutralPoint;
  LIMIT_MIN_MAX(StickGasHover, 70, 175); // reserve some range for trim up and down
}

void ChannelAssingment(void)
{
cli();
	ChannelNick = PPM_in[EE_Parameter.Kanalbelegung[K_NICK]];
	ChannelRoll = PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]];
	ChannelYaw = PPM_in[EE_Parameter.Kanalbelegung[K_GIER]];
	ChannelGas = PPM_in[EE_Parameter.Kanalbelegung[K_GAS]];
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+ Analoge Steuerung per Seriell
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if((ExternalControl.Config & EC_VALID) && (Parameter_ExternalControl > 128))
    {
		ChannelNick += ExternalControl.Nick;
		ChannelRoll += ExternalControl.Roll;
		ChannelYaw += ExternalControl.Gier;
     //This change is serious but avoids the requirement of putting the remote gas to a high
     // position in order to get external control working, because this would lead to immediate
     // rising after deactivation of external control
     //if(ExternControl.Gas < StickGas)
     ChannelGas = ExternalControl.Gas;
//        if(ExternalControl.Config & EC_GAS_ADD) ChannelGas += ExternalControl.Gas;
//		else
//		 {
//		  if(ExternalControl.Gas < ChannelGas) ChannelGas = ExternalControl.Gas; // the RC-Stick is the MAX value here
//	     }
	}
sei();
	if(ChannelNick > 127) ChannelNick = 127; 	else if(ChannelNick < -127) ChannelNick = -127;
	if(ChannelRoll > 127) ChannelRoll = 127; 	else if(ChannelRoll < -127) ChannelRoll = -127;
	if(ChannelYaw  > 127) ChannelYaw = 127; 	else if(ChannelYaw < -127) ChannelYaw = -127;
	if(ChannelGas  > 127) ChannelGas = 127; 	else if(ChannelGas < -127) ChannelGas = -127;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Calibration required before motors can be armed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void preFlightCalibration(unsigned char setting)
{
    ParamSet_ReadFromEEProm(ActiveParamSet);
    LipoDetection(0);
    LIBFC_ReceiverInit(EE_Parameter.Receiver);
    if((Parameter_GlobalConfig & CFG_HOEHENREGELUNG))  // H�henregelung aktiviert?
    {
     if((MessLuftdruck > 950) || (MessLuftdruck < 750)) SucheLuftruckOffset();
    }
    if(!setting && ChannelRoll < -70 && ChannelNick < 70) CalibrationDone = SetNeutral(3); // Boat-Mode
    //						   else
    //						   if(!setting && ChannelYaw < -75 && abs(ChannelNick) < 70)  CalibrationDone = SetNeutral(2); // store ACC values into EEPROM
    else	   CalibrationDone = SetNeutral(1);
    ServoActive = 1;
    DDRD  |=0x80; // enable J7 -> Servo signal
    #if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
    if(VersionInfo.HardwareError[0]) SpeakHoTT = SPEAK_ERR_SENSOR; 
    else 
    if(!CalibrationDone) SpeakHoTT = SPEAK_ERR_CALIBARTION; 
    else SpeakHoTT = SPEAK_CALIBRATE; 
    ShowSettingNameTime = 10; // for HoTT & Jeti 
    #endif
    Piep(ActiveParamSet,120);
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Calibration with storing acc data in eeprom
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void persistedCalibration(void)
{
    CalibrationDone = SetNeutral(2); // store ACC values into EEPROM
    ServoActive = 1;
    DDRD  |=0x80; // enable J7 -> Servo signal
    #if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
    if(VersionInfo.HardwareError[0]) SpeakHoTT = SPEAK_ERR_SENSOR; 
    else 
    if(!CalibrationDone) SpeakHoTT = SPEAK_ERR_CALIBARTION; 
    else SpeakHoTT = SPEAK_CALIBRATE; 
    ShowSettingNameTime = 10; // for HoTT & Jeti 
    #endif
    Piep(ActiveParamSet,120);
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Einschalten
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char armMotors(void)
{
  if (CalibrationDone)
    FC_StatusFlags |= FC_STATUS_START;

  StartLuftdruck = Luftdruck;
  HoehenWertF = 0;
  HoehenWert = 0;
  SummenHoehe = 0;
  
if(FC_StatusFlags3 & FC_STATUS3_BOAT) { if((abs(MesswertGier) > 32*2 || abs(MesswertNick) > 20*3) || abs(MesswertRoll) > 20*3)  CalibrationDone = 0; } // dann ist der Gyro defekt, schlecht kalibriert oder der MK dreht sich
else
								if((abs(MesswertGier) > 32 || abs(MesswertNick) > 20) || abs(MesswertRoll) > 20)  CalibrationDone = 0; // dann ist der Gyro defekt, schlecht kalibriert oder der MK dreht sich
    if (!VersionInfo.HardwareError[0] && CalibrationDone && !NC_ErrorCode)
    {
      modell_fliegt = 1;
      MotorenEin = 1;
      Mess_Integral_Gier = 0;
      Mess_Integral_Gier2 = 0;
	  Mess_IntegralNick = EE_Parameter.GyroAccFaktor * (long)Mittelwert_AccNick;
      Mess_IntegralRoll = EE_Parameter.GyroAccFaktor * (long)Mittelwert_AccRoll;
      SummeNick = 0;
      SummeRoll = 0;
      //									ControlHeading = (((int) EE_Parameter.OrientationAngle * 15 + KompassValue) % 360) / 2;
      NeueKompassRichtungMerken = 100; // 2 sekunden
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
      SpeakHoTT = SPEAK_STARTING; 
#endif
      return 1;
    }
    else
    {
    #if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
      if(!CalibrationDone) SpeakHoTT = SPEAK_ERR_CALIBARTION; 
    #endif
      beeptime = 1500; // indicate missing calibration
      return 0;
    }
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Ausschalten
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void unarmMotors(void)
{
  SummeNick = 0;
  SummeRoll = 0;
  StickNick = 0;
  StickRoll = 0;

  MotorenEin = 0;
  modell_fliegt = 0;
  FC_StatusFlags2 &= ~(FC_STATUS2_WAIT_FOR_TAKEOFF | FC_STATUS2_AUTO_STARTING | FC_STATUS2_AUTO_LANDING);
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
  SpeakHoTT = SPEAK_MK_OFF;
#endif

}

//############################################################################
//
void MotorRegler(void)
//############################################################################
{
	 int pd_ergebnis_nick,pd_ergebnis_roll,tmp_int, tmp_int2;
	 int GierMischanteil,GasMischanteil;
     static long sollGier = 0,tmp_long,tmp_long2;
  	 static unsigned int RcLostTimer, delay_Acc_neutral;
  	 static unsigned char delay_neutral = 0;
  	 static unsigned char delay_einschalten = 0,delay_ausschalten = 0;
	 static signed char move_safety_switch = 0;
     int IntegralNickMalFaktor,IntegralRollMalFaktor;
	 unsigned char i;
	Mittelwert();
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gaswert ermitteln
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(!(FC_StatusFlags & (FC_STATUS_EMERGENCY_LANDING | FC_STATUS2_RC_FAILSAVE_ACTIVE)))
   {
	if(EE_Parameter.GlobalConfig3 & CFG3_VARIO_FAILSAFE) 
	{
     if(HoverGas && HoverGas < 150 * STICK_GAIN)
	   {
	  	HooverGasEmergencyPercent = (HoverGas/(STICK_GAIN) * EE_Parameter.NotGas) / 100; // i.e. 80% of Hovergas
	   }
      else HooverGasEmergencyPercent = 45;  // default if the Hoovergas was could not calculated yet
    } else HooverGasEmergencyPercent = EE_Parameter.NotGas;
   }
   if(GasIsZeroCnt == 30000)  // in that case we have RC-Lost, but the MK is probably landed
    {
	 StickGas = 0; // Hold Gas down in that case 
//	 ExternalControl.Gas = 0;
	 HooverGasEmergencyPercent = MIN_GAS;
	} 
   	GasMischanteil = StickGas;
    if(GasMischanteil < MIN_GAS + 10) GasMischanteil = MIN_GAS + 10;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Empfang schlecht
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if(SenderOkay < 100 && !(FC_StatusFlags2 & FC_STATUS2_RC_FAILSAVE_ACTIVE))
        {
		ServoFailsafeActive = SERVO_FS_TIME;
        if(RcLostTimer) RcLostTimer--;
        else
         {
          MotorenEin = 0;
 		  modell_fliegt = 0;
          FC_StatusFlags &= ~(FC_STATUS_EMERGENCY_LANDING | FC_STATUS_FLY);
         }
        ROT_ON;
        if(modell_fliegt > 1000 && Capacity.MinOfMaxPWM > 100)  // wahrscheinlich in der Luft --> langsam absenken
            {
            GasMischanteil = HooverGasEmergencyPercent;
            FC_StatusFlags |= FC_STATUS_EMERGENCY_LANDING;
            }
         else 
		    {
			  MotorenEin = 0;
			}  
        }
        else
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Emfang gut
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if(SenderOkay > 140)
            {
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
		     static unsigned int trigger = 1000;
			 static unsigned char old_switch = 100;
			if(EE_Parameter.StartLandChannel && EE_Parameter.LandingSpeed)
			{
				if(PPM_in[EE_Parameter.StartLandChannel] > 50) 
				{
				 if(old_switch == 50) if(FC_StatusFlags2 & FC_STATUS2_WAIT_FOR_TAKEOFF) { FC_StatusFlags2 |= FC_STATUS2_AUTO_STARTING; SpeakHoTT = SPEAK_RISING;}
				 FC_StatusFlags2 &= ~FC_STATUS2_AUTO_LANDING;
				 old_switch = 150;
				}
				else 
				if(PPM_in[EE_Parameter.StartLandChannel] < -50) 
				{
				 if(old_switch == 150) { FC_StatusFlags2 |= FC_STATUS2_AUTO_LANDING; SpeakHoTT = SPEAK_SINKING;}
 				 FC_StatusFlags2 &= ~FC_STATUS2_AUTO_STARTING;
				 old_switch = 50;
				}
				else 
				{
				 FC_StatusFlags2 &= ~(FC_STATUS2_AUTO_STARTING | FC_STATUS2_AUTO_LANDING);
				}
			}
#endif
			FC_StatusFlags &= ~FC_STATUS_EMERGENCY_LANDING;
            RcLostTimer = EE_Parameter.NotGasZeit * 50;
            if(GasMischanteil > 40 && MotorenEin)
                {
                if(modell_fliegt < 0xffff) modell_fliegt++;
                }
            if((modell_fliegt < 256))
             {
                SummeNick = 0;
                SummeRoll = 0;
                sollGier = 0;
                Mess_Integral_Gier = 0;
				FC_StatusFlags2 |= FC_STATUS2_WAIT_FOR_TAKEOFF;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				old_switch = 100;
#endif
             } 
			else 
			 {
               FC_StatusFlags |= FC_STATUS_FLY;
			   if(FC_StatusFlags2 & FC_STATUS2_WAIT_FOR_TAKEOFF)
			   {
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				if((NC_To_FC_Flags & NC_TO_FC_AUTOSTART || FC_StatusFlags2 & FC_STATUS2_AUTO_STARTING) && (VarioCharacter == '=') && ACC_AltitudeControl)
				{
				   FromNC_AltitudeSpeed = 80;
				   FromNC_AltitudeSetpoint = 500;
				   SollHoehe = 500;
				   trigger = 1000;
				   if(NC_To_FC_Flags & NC_TO_FC_AUTOSTART) SpeakHoTT = SPEAK_NEXT_WP; 
/*				   if(StartTrigger != 2)
						{
						 StartTrigger = 1;
						 if(HoverGas < STICK_GAIN * 35) HoverGas = STICK_GAIN * 35;
						}
*/
			    }
//				else FC_StatusFlags2 &= ~(FC_STATUS2_AUTO_STARTING);
#endif
  			    if(HoehenWertF > 150 || HoehenWert < -350 || !(Parameter_GlobalConfig & CFG_HOEHENREGELUNG)) 
				 {
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				  trigger = 1000;
				  if(FC_StatusFlags2 & FC_STATUS2_AUTO_STARTING) { FromNC_AltitudeSpeed = 0; SollHoehe = 300;/*HoehenWertF + 100;*/}
				  else SpeakHoTT = SPEAK_RISING;
#endif
				  FC_StatusFlags2 &= ~(FC_STATUS2_WAIT_FOR_TAKEOFF | FC_STATUS2_AUTO_STARTING | FC_STATUS2_AUTO_LANDING);
				 }
                SummeNick = 0;
                SummeRoll = 0;
                Mess_Integral_Gier = 0;
//				sollGier = 0;
				if(modell_fliegt > 1000) modell_fliegt = 1000;  // for the Hooverpoint-Estimation
			   }
				else  // Flying mode
				{
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				if((FC_StatusFlags2 & FC_STATUS2_AUTO_LANDING) && (VarioCharacter == 'v' || VarioCharacter == '=') && ACC_AltitudeControl)
				{
				   FromNC_AltitudeSpeed = EE_Parameter.LandingSpeed;
				   FromNC_AltitudeSetpoint = -20000;
			    }
 				 if(trigger < 1000) 
					{
						trigger++;
						SummeNick = 0;
						SummeRoll = 0;
						Mess_Integral_Gier = 0;
						SollHoehe = HoehenWertF - 300;
						if(trigger == 1000 && FC_StatusFlags2 & FC_STATUS2_AUTO_LANDING && VarioCharacter != '+') 
						 {
						   FC_StatusFlags2 &= ~FC_STATUS2_AUTO_LANDING; 
						   FC_StatusFlags2 |= FC_STATUS2_WAIT_FOR_TAKEOFF;  // go back into starting state
					     }	
					}
				 else 
  				 if(ACC_AltitudeControl && (VarioCharacter == 'v' || VarioCharacter == '-') && HoehenWert < 1000 /*&& FromNC_AltitudeSetpoint < 0*/)
				  {
					if((Aktuell_az/4 > EE_Parameter.LandingPulse) && (NeutralAccZ/4 > EE_Parameter.LandingPulse))
					 { 
					  trigger = 0; 
					  SpeakHoTT = SPEAK_LANDING; 
					 };
				  }
#endif
               }
			  }  // end of: modell_fliegt > 256			   
            if((ChannelGas > 80) && MotorenEin == 0 && !(NC_To_FC_Flags & NC_TO_FC_SIMULATION_ACTIVE)) 
                {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// auf Nullwerte kalibrieren
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                if(ChannelYaw > 75)  // Neutralwerte
                    {
                    if(++delay_neutral > 200)  // nicht sofort
                        {
                        unsigned char setting = 0;
                        delay_neutral = 0;
                        modell_fliegt = 0;
                        if(ChannelNick > 70 || abs(ChannelRoll) > 70)
                        {
                         if(ChannelRoll > 70 && ChannelNick < 70 && ChannelNick > -70) setting = 1;
                         if(ChannelRoll > 70 && ChannelNick > 70) setting = 2;
                         if(ChannelRoll < 70 && ChannelNick > 70) setting = 3;
                         if(ChannelRoll <-70 && ChannelNick > 70) setting = 4;
                         if(ChannelRoll <-70 && ChannelNick < 70  && ChannelNick > -70) setting = 5;
                         if(setting) SetActiveParamSet(setting);  // aktiven Datensatz merken
                        }
                         if(abs(ChannelRoll) < 30 && ChannelNick < -70)
                          {
                           WinkelOut.CalcState = 1; // Compass kalibrieren
						   CalibrationDone = 0;
                           beeptime = 1000;
                          }
                          else
                          {
                            preFlightCalibration(setting);
                         }
                        }
                    }
					else
                    if(ChannelYaw < -75 && abs(ChannelRoll) < 16 && abs(ChannelRoll) < 16)  // ACC calibrate
                     {                      
                     if(++delay_Acc_neutral > 500 * 5)  // 5 sekunden
                        {
                          delay_Acc_neutral = 0; 
						  persistedCalibration();
                         }
                     }
                  else { delay_neutral = 0; delay_Acc_neutral = 0;};
                }
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gas ist unten
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            if(ChannelGas < -100)
                {
					if(PPM_diff[EE_Parameter.MotorSafetySwitch & 127] > 5) move_safety_switch = 100;
					else
					if(PPM_diff[EE_Parameter.MotorSafetySwitch & 127] < -5) move_safety_switch = -100;
					// Motoren Starten
					if(!MotorenEin)
                	{
						if(((((ChannelYaw < -100) && ((!(EE_Parameter.GlobalConfig3 & CFG3_MOTOR_SWITCH_MODE) && PPM_in[EE_Parameter.MotorSafetySwitch] < -75) || EE_Parameter.MotorSafetySwitch == 0)))
						|| (((EE_Parameter.GlobalConfig3 & CFG3_MOTOR_SWITCH_MODE) && PPM_in[EE_Parameter.MotorSafetySwitch] > -10 && move_safety_switch == 100))) 
						&& !(NC_To_FC_Flags & NC_TO_FC_SIMULATION_ACTIVE))  
						{
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Einschalten
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
							if(CalibrationDone) FC_StatusFlags |= FC_STATUS_START;

							StartLuftdruck = Luftdruck;
							HoehenWert = 0;
							HoehenWert_Mess = 0;
							GasIsZeroCnt = 600;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
							HoehenWertF_Mess = 0;
#endif
							SummenHoehe = 0;
							if((ChannelNick > -100 || abs(ChannelRoll) < 100) && EE_Parameter.MotorSafetySwitch == 0) delay_einschalten = 0;
							if(++delay_einschalten > 253)
							{
  if (armMotors()){
    sollGier = 0;
  }
							}
						}
						else delay_einschalten = 0;
					}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Auschalten
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
					else // only if motors are running
					{
//						if((PPM_in[EE_Parameter.Kanalbelegung[K_GIER]] > 75) && (PPM_in[EE_Parameter.MotorSafetySwitch] < -75 || EE_Parameter.MotorSafetySwitch == 0))
						if((((ChannelYaw > 100) && ((!(EE_Parameter.GlobalConfig3 & CFG3_MOTOR_SWITCH_MODE) && PPM_in[EE_Parameter.MotorSafetySwitch] < -75) || EE_Parameter.MotorSafetySwitch == 0)))
						|| (((EE_Parameter.GlobalConfig3 & CFG3_MOTOR_SWITCH_MODE) && PPM_in[EE_Parameter.MotorSafetySwitch] < -50 && move_safety_switch == -100)))
						{
							if((ChannelNick > -100 || abs(ChannelRoll) < 100) && EE_Parameter.MotorSafetySwitch == 0)
							{
								delay_ausschalten = 0;
							}
							else 
							{
							  SummeNick = 0;
							  SummeRoll = 0;
							  StickNick = 0;
							  StickRoll = 0;
							} 
							if(++delay_ausschalten > 250)  // nicht sofort
							{
                                delay_ausschalten = 0;
                                unarmMotors();
							}
							else 
							 if(delay_ausschalten == 100) beeptime = 3500;
						}
						else delay_ausschalten = 0;
					}
                  if(GasIsZeroCnt < 1000) 
				   {
				    if(VarioMeter > -150) GasIsZeroCnt++;
					else if(GasIsZeroCnt) GasIsZeroCnt--;
				   }	
				}
				else // gas not at minimum
				{
				  move_safety_switch = 0; 
				  GasIsZeroCnt = 0;
				}  
            }
			else  // Empfang zwischen 100 und 140 -> schlecht
			{
			if(GasIsZeroCnt >= 750)  // gas-stick was down for 1.5 seconds before RC-Lost
			 {
			   if((GPSInfo.HomeDistance < 40 * 10) && (HoehenWert < 15 * 100))  // and we are at the starting point -> maybe landed? 
				{
				 GasIsZeroCnt = 30000;
				 if(modell_fliegt > 1001) modell_fliegt = 1001;
				}	 
			 }
			}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// neue Werte von der Funke
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

 if(!NewPpmData-- || (FC_StatusFlags & FC_STATUS_EMERGENCY_LANDING))
  {
	static int stick_nick,stick_roll;
	unsigned char stick_p;
    ParameterZuordnung();
	ChannelAssingment();

	stick_p = EE_Parameter.Stick_P;
    stick_nick = (stick_nick * 3 + ChannelNick * stick_p) / 4;
    stick_roll = (stick_roll * 3 + ChannelRoll * stick_p) / 4;
cli();
    stick_nick += PPM_diff[EE_Parameter.Kanalbelegung[K_NICK]] * EE_Parameter.Stick_D;
    stick_roll += PPM_diff[EE_Parameter.Kanalbelegung[K_ROLL]] * EE_Parameter.Stick_D;
sei();
   	StickGas  = ChannelGas + 127;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// CareFree und freie Wahl der vorderen Richtung
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	if(CareFree)
	{
		signed int nick, roll;
		nick = stick_nick / 4;
		roll = stick_roll / 4;
		StickNick = ((FromNC_Rotate_C * nick) + (FromNC_Rotate_S * roll)) / (32 / 4);
		StickRoll = ((FromNC_Rotate_C * roll) - (FromNC_Rotate_S * nick)) / (32 / 4);
	}
	else
	{
		FromNC_Rotate_C = sintab[EE_Parameter.OrientationAngle + 6];
		FromNC_Rotate_S = sintab[EE_Parameter.OrientationAngle];
		StickNick = ((FromNC_Rotate_C * stick_nick) + (FromNC_Rotate_S * stick_roll)) / 8;
		StickRoll = ((FromNC_Rotate_C * stick_roll) - (FromNC_Rotate_S * stick_nick)) / 8;
	}

    StickGier = -ChannelYaw;
	if(StickGier > 4) StickGier -= 4; 	else
	if(StickGier < -4) StickGier += 4; else StickGier = 0;

if(GasIsZeroCnt > 512) // About to switch - off
 {
  StickNick = StickNick/8; 
  StickRoll = StickRoll/8;  
  SummeNick = 0;
  SummeRoll = 0;
 }
else
    if(GPS_Aid_StickMultiplikator) // in that case the GPS controls stronger
	 { 
	  StickNick = (GPS_Aid_StickMultiplikator * (StickNick / 8)) / 16; 
	  StickRoll = (GPS_Aid_StickMultiplikator * (StickRoll / 8)) / 16; 
	 }
    StickNick -= GPS_Nick;
    StickRoll -= GPS_Roll;

    GyroFaktor     = (Parameter_Gyro_P + 10.0);
    IntegralFaktor = Parameter_Gyro_I;
    GyroFaktorGier     = (Parameter_Gyro_Gier_P + 10.0);
    IntegralFaktorGier = Parameter_Gyro_Gier_I;


    if(Parameter_GlobalConfig & CFG_HEADING_HOLD) IntegralFaktor =  0;

    if(abs(StickNick/STICK_GAIN) > MaxStickNick)
     {
      MaxStickNick = abs(StickNick)/STICK_GAIN;
      if(MaxStickNick > 100) MaxStickNick = 100;
     }
     else MaxStickNick--;
    if(abs(StickRoll/STICK_GAIN) > MaxStickRoll)
     {
      MaxStickRoll = abs(StickRoll)/STICK_GAIN;
      if(MaxStickRoll > 100) MaxStickRoll = 100;
     }
     else MaxStickRoll--;
    if(FC_StatusFlags & FC_STATUS_EMERGENCY_LANDING)  {MaxStickNick = 0; MaxStickRoll = 0;}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Looping?
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if((PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]] > EE_Parameter.LoopThreshold) && EE_Parameter.BitConfig & CFG_LOOP_LINKS)  Looping_Links = 1;
  else
   {
     {
      if((PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]] < (EE_Parameter.LoopThreshold - EE_Parameter.LoopHysterese))) Looping_Links = 0;
     }
   }
  if((PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]] < -EE_Parameter.LoopThreshold) && EE_Parameter.BitConfig & CFG_LOOP_RECHTS) Looping_Rechts = 1;
   else
   {
   if(Looping_Rechts) // Hysterese
     {
      if(PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]] > -(EE_Parameter.LoopThreshold - EE_Parameter.LoopHysterese)) Looping_Rechts = 0;
     }
   }

  if((PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] > EE_Parameter.LoopThreshold) && EE_Parameter.BitConfig & CFG_LOOP_OBEN) Looping_Oben = 1;
  else
   {
    if(Looping_Oben)  // Hysterese
     {
      if((PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] < (EE_Parameter.LoopThreshold - EE_Parameter.LoopHysterese))) Looping_Oben = 0;
     }
   }
  if((PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] < -EE_Parameter.LoopThreshold) && EE_Parameter.BitConfig & CFG_LOOP_UNTEN) Looping_Unten = 1;
   else
   {
    if(Looping_Unten) // Hysterese
     {
      if(PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] > -(EE_Parameter.LoopThreshold - EE_Parameter.LoopHysterese)) Looping_Unten = 0;
     }
   }

   if(Looping_Links || Looping_Rechts)   Looping_Roll = 1; else Looping_Roll = 0;
   if(Looping_Oben  || Looping_Unten) {  Looping_Nick = 1; Looping_Roll = 0; Looping_Links = 0; Looping_Rechts = 0;} else Looping_Nick = 0;
  } // Ende neue Funken-Werte

  if(Looping_Roll || Looping_Nick)
   {
    if(GasMischanteil > EE_Parameter.LoopGasLimit) GasMischanteil = EE_Parameter.LoopGasLimit;
	TrichterFlug = 1;
   }


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Bei Empfangsausfall im Flug
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if(FC_StatusFlags2 & FC_STATUS2_RC_FAILSAVE_ACTIVE) 
   {
    StickNick = -GPS_Nick;
    StickRoll = -GPS_Roll;
   	StickGas = StickGasHover;
    StickGier = 0;
	Parameter_GlobalConfig &= ~(CFG_HEADING_HOLD | CFG_DREHRATEN_BEGRENZER);
	Parameter_GlobalConfig |= CFG_HOEHENREGELUNG | CFG_ACHSENKOPPLUNG_AKTIV | CFG_KOMPASS_AKTIV | CFG_GPS_AKTIV | CFG_HOEHEN_SCHALTER | CFG_GPS_AKTIV;
	Parameter_ExtraConfig &= ~(CFG2_HEIGHT_LIMIT | CFG_LEARNABLE_CAREFREE | CFG2_VARIO_BEEP);
	Parameter_HoehenSchalter = 200; // switch on
   }
   else 
   if(FC_StatusFlags & FC_STATUS_EMERGENCY_LANDING)
    {
     StickGier = 0;
     StickNick = 0;
     StickRoll = 0;
     GyroFaktor     = 90;
     IntegralFaktor = 120;
     GyroFaktorGier     = 90;
     IntegralFaktorGier = 120;
     Looping_Roll = 0;
     Looping_Nick = 0;
    }

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Integrale auf ACC-Signal abgleichen
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define ABGLEICH_ANZAHL 256L

 MittelIntegralNick  += IntegralNick;    // F�r die Mittelwertbildung aufsummieren
 MittelIntegralRoll  += IntegralRoll;
 if(Looping_Nick || Looping_Roll)
  {
    MittelIntegralNick = 0;
    MittelIntegralRoll = 0;
    ZaehlMessungen = 0;
    LageKorrekturNick = 0;
    LageKorrekturRoll = 0;
  }

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(!Looping_Nick && !Looping_Roll && (Aktuell_az > 512 || MotorenEin))
  {
   long tmp_long, tmp_long2;
   if(FromNaviCtrl_Value.Kalman_K > 0 /*&& !TrichterFlug*/)
     {
		tmp_long = (long)(IntegralNick / EE_Parameter.GyroAccFaktor - (long)(Mittelwert_AccNick - FromNaviCtrl.AccErrorN));
		tmp_long2 = (long)(IntegralRoll / EE_Parameter.GyroAccFaktor - (long)(Mittelwert_AccRoll - FromNaviCtrl.AccErrorR));
		tmp_long  = (tmp_long  * FromNaviCtrl_Value.Kalman_K) / (32 * 16);
		tmp_long2 = (tmp_long2 * FromNaviCtrl_Value.Kalman_K) / (32 * 16);
		if((MaxStickNick > 64) || (MaxStickRoll > 64))
		{
		tmp_long  /= 2;
		tmp_long2 /= 2;
		}
		if(tmp_long >  (long) FromNaviCtrl_Value.Kalman_MaxFusion)  tmp_long  = (long) FromNaviCtrl_Value.Kalman_MaxFusion;
		if(tmp_long <  (long)-FromNaviCtrl_Value.Kalman_MaxFusion)  tmp_long  = (long)-FromNaviCtrl_Value.Kalman_MaxFusion;
		if(tmp_long2 > (long) FromNaviCtrl_Value.Kalman_MaxFusion)  tmp_long2 = (long) FromNaviCtrl_Value.Kalman_MaxFusion;
		if(tmp_long2 < (long)-FromNaviCtrl_Value.Kalman_MaxFusion)  tmp_long2 = (long)-FromNaviCtrl_Value.Kalman_MaxFusion;
     }
     else
     {
		tmp_long = (long)(IntegralNick / EE_Parameter.GyroAccFaktor - (long)Mittelwert_AccNick);
		tmp_long2 = (long)(IntegralRoll / EE_Parameter.GyroAccFaktor - (long)Mittelwert_AccRoll);
		tmp_long /= 16;
		tmp_long2 /= 16;
		if((MaxStickNick > 64) || (MaxStickRoll > 64))
		{
		tmp_long  /= 3;
		tmp_long2 /= 3;
		}
		if(abs(ChannelYaw) > 25)
		{
		tmp_long  /= 3;
		tmp_long2 /= 3;
		}
		KompassFusion = 25;
#define AUSGLEICH  32
		if(tmp_long >  AUSGLEICH)  tmp_long  = AUSGLEICH;
		if(tmp_long < -AUSGLEICH)  tmp_long  =-AUSGLEICH;
		if(tmp_long2 > AUSGLEICH)  tmp_long2 = AUSGLEICH;
		if(tmp_long2 <-AUSGLEICH)  tmp_long2 =-AUSGLEICH;
     }

	Mess_IntegralNick -= tmp_long;
	Mess_IntegralRoll -= tmp_long2;
	DriftNick += tmp_long;
	DriftRoll += tmp_long2;
  }
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 if(ZaehlMessungen >= ABGLEICH_ANZAHL)  //alle 512ms
 {
//  static int cnt = 0;
//  static char last_n_p,last_n_n,last_r_p,last_r_n;
  static long MittelIntegralNick_Alt,MittelIntegralRoll_Alt;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gyro-Drift ermitteln
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(EE_Parameter.Driftkomp && abs(Mittelwert_AccNick) < 200*4 && abs(Mittelwert_AccRoll) < 200*4 && !TrichterFlug && abs(MesswertGier) < 32/* && (FC_StatusFlags & FC_STATUS_FLY)*/)
  {
   DriftNick -= DriftNick / (64 * (unsigned int) EE_Parameter.Driftkomp);
   DriftRoll -= DriftRoll / (64 * (unsigned int) EE_Parameter.Driftkomp);
   GierGyroFehler -= GierGyroFehler / (64 * (unsigned int) EE_Parameter.Driftkomp);
   if((MaxStickNick > 64) || (MaxStickRoll > 64) || (abs(ChannelYaw) > 25))
    {
     DriftNick /= 2;
     DriftRoll /= 2;
	 GierGyroFehler = 0;
    }
	 if(DriftNick > 3000) { DriftNick = 0; AdNeutralNick++;}
	 if(DriftNick <-3000) { DriftNick = 0; AdNeutralNick--;}
 	 if(DriftRoll > 3000) { DriftRoll = 0; AdNeutralRoll++;}
	 if(DriftRoll <-3000) { DriftRoll = 0; AdNeutralRoll--;}
     if(GierGyroFehler > 3500) { GierGyroFehler = 0; AdNeutralGier++; }
     if(GierGyroFehler <-3500) { GierGyroFehler = 0; AdNeutralGier--; }
  }
  else
  {
   DriftNick = 0;
   DriftRoll = 0;
   GierGyroFehler = 0;
  }
  TrichterFlug = 0;

  if(!IntegralFaktor) { LageKorrekturRoll = 0; LageKorrekturNick = 0;} // z.B. bei HH
// +++++++++++++++++++++++++++++++++++++++++++++++++++++
   MittelIntegralNick_Alt = MittelIntegralNick;
   MittelIntegralRoll_Alt = MittelIntegralRoll;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++
    MittelIntegralNick = 0;
    MittelIntegralRoll = 0;
    ZaehlMessungen = 0;
 } //  ZaehlMessungen >= ABGLEICH_ANZAHL

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Gieren
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(abs(StickGier) > 3) // war 15
     {
//      KompassSignalSchlecht = 1000;
      if(!(Parameter_GlobalConfig & CFG_KOMPASS_FIX))
       {
         NeueKompassRichtungMerken = 50; // eine Sekunde zum Einloggen
        };
     }
    tmp_int  = (long) EE_Parameter.StickGier_P * ((long)StickGier * abs(StickGier)) / 512L; // expo  y = ax + bx�
    tmp_int += (EE_Parameter.StickGier_P * StickGier) / 4;
	if(GasIsZeroCnt > 512 && ! ((ExternalControl.Config & EC_VALID) && (Parameter_ExternalControl > 128)))
    {
        tmp_int = 0; // disable Yawing when Gas-Stick is to Zero
    }
	tmp_int += CompassGierSetpoint;
    sollGier = tmp_int;
    Mess_Integral_Gier -= tmp_int;
    if(Mess_Integral_Gier > 50000) Mess_Integral_Gier = 50000;  // begrenzen
    if(Mess_Integral_Gier <-50000) Mess_Integral_Gier =-50000;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Kompass
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(KompassValue >= 0 && (Parameter_GlobalConfig & CFG_KOMPASS_AKTIV))
     {
      if(CalculateCompassTimer-- == 1)
	  {
       int w,v,r,fehler,korrektur; // wird von der SPI-Routine auf 1 gesetzt
       CalculateCompassTimer = 13; // falls keine Navi-Daten
	   // max. Korrekturwert sch�tzen
       w = abs(IntegralNick /512); // mit zunehmender Neigung den Einfluss drosseln
       v = abs(IntegralRoll /512);
       if(v > w) w = v; // gr�sste Neigung ermitteln
//       korrektur = w / 4 + 1;
	   korrektur = w / 8 + 2;
	   ErsatzKompassInGrad = ErsatzKompass/GIER_GRAD_FAKTOR;
	   // Kompassfehlerwert bestimmen   
	   fehler = ((540 + KompassValue - ErsatzKompassInGrad) % 360) - 180;
	   // GIER_GRAD_FAKTOR ist ca. 1200
	   // Kompasswert einloggen
       if(NeueKompassRichtungMerken) KompassSollWert = ErsatzKompassInGrad;

       if(KompassSignalSchlecht) KompassSignalSchlecht--;
	   else 
       if(w < 25)
        {
        GierGyroFehler += fehler;
		if(NeueKompassRichtungMerken) NeueKompassRichtungMerken--;
        }
       // Kompass fusionieren
       if(!KompassSignalSchlecht) ErsatzKompass += (fehler * KompassFusion) / korrektur;
       // MK Gieren
	   if(!NeueKompassRichtungMerken)
       {
           r = ((540 + (KompassSollWert - ErsatzKompassInGrad)) % 360) - 180;
           v = r * (Parameter_KompassWirkung/2);  // nach Kompass ausrichten
		   CompassGierSetpoint = v / 16;
       }
      else CompassGierSetpoint = 0; 
      } // CalculateCompassTimer
     }
	 else CompassGierSetpoint = 0; 

//DebugOut.Analog[16] = KompassFusion;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Drehgeschwindigkeit und -winkel zu einem Istwert zusammenfassen
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(TrichterFlug)  { SummeRoll = 0; SummeNick = 0;};

  if(!Looping_Nick) IntegralNickMalFaktor = (IntegralNick * IntegralFaktor) /  (44000 / STICK_GAIN); else IntegralNickMalFaktor = 0;
  if(!Looping_Roll) IntegralRollMalFaktor = (IntegralRoll * IntegralFaktor) /  (44000 / STICK_GAIN); else IntegralRollMalFaktor = 0;

#define TRIM_MAX 200
 if(TrimNick > TRIM_MAX) TrimNick = TRIM_MAX; else  if(TrimNick <-TRIM_MAX) TrimNick =-TRIM_MAX;
 if(TrimRoll > TRIM_MAX) TrimRoll = TRIM_MAX; else  if(TrimRoll <-TRIM_MAX) TrimRoll =-TRIM_MAX;

    MesswertNick = IntegralNickMalFaktor + (long)((long)MesswertNick * GyroFaktor + (long)TrimNick * 128L) / (256L / STICK_GAIN);
    MesswertRoll = IntegralRollMalFaktor + (long)((long)MesswertRoll * GyroFaktor + (long)TrimRoll * 128L) / (256L / STICK_GAIN);
    MesswertGier = (long)(MesswertGier * 2 * (long)GyroFaktorGier) / (256L / STICK_GAIN) + (long)(Integral_Gier * IntegralFaktorGier) / (2 * (44000 / STICK_GAIN));

    // Maximalwerte abfangen
    #define MAX_SENSOR  (4096)
    if(MesswertNick >  MAX_SENSOR) MesswertNick =  MAX_SENSOR;
    if(MesswertNick < -MAX_SENSOR) MesswertNick = -MAX_SENSOR;
    if(MesswertRoll >  MAX_SENSOR) MesswertRoll =  MAX_SENSOR;
    if(MesswertRoll < -MAX_SENSOR) MesswertRoll = -MAX_SENSOR;
    if(MesswertGier >  MAX_SENSOR) MesswertGier =  MAX_SENSOR;
    if(MesswertGier < -MAX_SENSOR) MesswertGier = -MAX_SENSOR;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Undervoltage
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if(!(FC_StatusFlags & FC_STATUS_LOWBAT))
  {
    GasMischanteil = ((unsigned int)GasMischanteil * BattLowVoltageWarning) / UBat; // Gas auf das aktuelle Spannungvieveau beziehen
  } 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Auto-Landing
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 static unsigned char slower;
 if(!slower--)
  {
   static unsigned int u_filter = 0;
   if(!u_filter) u_filter = UBat; 
   if(UBat > u_filter) u_filter++; else 
   if(UBat < u_filter) u_filter--; 
   slower = 100; // 5Hz
   if(u_filter < BattAutoLandingVoltage)
	 {
	  LowVoltageLandingActive = 10; // 2 sek
	  ServoFailsafeActive = SERVO_FS_TIME;
	 }
	 else if(u_filter > BattAutoLandingVoltage + LipoCells && LowVoltageLandingActive) LowVoltageLandingActive--; 

   if(u_filter < BattComingHomeVoltage)
	 {
	  LowVoltageHomeActive = 25; // min. 5 sek
	 }
	 else if(u_filter > BattComingHomeVoltage + LipoCells && LowVoltageHomeActive) LowVoltageHomeActive--; 
  }
  if(LowVoltageLandingActive && FromNC_AltitudeSetpoint >= 0) 
     {
		FromNC_AltitudeSpeed = EE_Parameter.LandingSpeed;
		if(HoehenWert > 15*100) FromNC_AltitudeSpeed *= 2; // faster above 15m
		FromNC_AltitudeSetpoint = -20000;
	 } 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// send SPI pending bytes
 if(BytegapSPI == 0)  SPI_TransmitByte(); 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// H�henregelung
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  GasMischanteil *= STICK_GAIN;
// if height control is activated
 if((Parameter_GlobalConfig & CFG_HOEHENREGELUNG) && !(Looping_Roll || Looping_Nick) && !(VersionInfo.HardwareError[0] & 0x7F))  // H�henregelung
	{
		#define HOVER_GAS_AVERAGE 16384L		// 16384 * 2ms = 32s averaging
		#define HC_GAS_AVERAGE 4    			// 4 * 2ms= 8ms averaging

		int HCGas, GasReduction = 0;
		static int HeightTrimming = 0;  // rate for change of height setpoint
		static int HeightDeviation = 0, FilterHCGas = 0;
		static unsigned long HoverGasFilter = 0;
		static unsigned char delay = 100, BaroAtUpperLimit = 0, BaroAtLowerLimit = 0;
        // Expand the measurement
		// measurement of air pressure close to upper limit and no overflow in correction of the new OCR0A value occurs
          if(!BaroExpandActive)
		   {
			if(MessLuftdruck > 920)
			{   // increase offset
             if(OCR0A < (255 - OPA_OFFSET_STEP))
			   {
				ExpandBaro -= 1;
				OCR0A = DruckOffsetSetting - OPA_OFFSET_STEP * ExpandBaro; // increase offset to shift ADC down
				OCR0B = 255 - OCR0A;
				beeptime = 300;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				if(ACC_AltitudeControl) BaroExpandActive = 50;
				else
#endif
                BaroExpandActive = 350;
				CalcExpandBaroStep();
			   }
			   else
			   {
			    BaroAtLowerLimit = 1;
               }
			}
			// measurement of air pressure close to lower limit and
			else
			if(MessLuftdruck < 100)
			{   // decrease offset
			  if(OCR0A > OPA_OFFSET_STEP)
			   {
				ExpandBaro += 1;
				OCR0A = DruckOffsetSetting - OPA_OFFSET_STEP * ExpandBaro; // decrease offset to shift ADC up
				OCR0B = 255 - OCR0A;
				beeptime = 300;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				if(ACC_AltitudeControl) BaroExpandActive = 50;
				else
#endif
                BaroExpandActive = 350;
				CalcExpandBaroStep();
			   }
			   else
			   {
			    BaroAtUpperLimit = 1;
               }
			}
			else
			{
			    BaroAtUpperLimit = 0;
				BaroAtLowerLimit = 0;
			}
		   }
		   else // delay, because of expanding the Baro-Range
		   {
		    // now clear the D-values
			  VarioMeter = 0;
cli();
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			  if(ACC_AltitudeControl) ACC_AltitudeFusion(1); // init
			  else SummenHoehe = HoehenWert * SM_FILTER;
#else 
              SummenHoehe = HoehenWert * SM_FILTER;
#endif
sei();
			  BaroExpandActive--;
		   }
		// if height control is activated by an rc channel
        if(Parameter_GlobalConfig & CFG_HOEHEN_SCHALTER)  // Regler wird �ber Schalter gesteuert
		{	// check if parameter is less than activation threshold
			if(Parameter_HoehenSchalter < 50) // for 3 or 2-state switch height control is disabled in lowest position
			{   //height control not active
				if(!delay--)
				{
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				    if(!SpeakHoTT && HoehenReglerAktiv)  SpeakHoTT = SPEAK_ALTITUDE_OFF; 
#endif
					HoehenReglerAktiv = 0; // disable height control
					SollHoehe = HoehenWert;  // update SetPoint with current reading
					delay = 1;
				}
			}
			else
			if(Parameter_HoehenSchalter > 70)
			{	//height control is activated
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			    if(!SpeakHoTT && !HoehenReglerAktiv)  SpeakHoTT = SPEAK_ALTITUDE_ON; 
#endif
				delay = 200;
				HoehenReglerAktiv = 1; // enable height control
			}
		}
		else // no switchable height control
		{
			SollHoehe = (/*(int16_t) ExternHoehenValue + */(int16_t) Parameter_HoehenSchalter) * (int)EE_Parameter.Hoehe_Verstaerkung;
			HoehenReglerAktiv = 1;
		}
		// calculate cos of nick and roll angle used for projection of the vertical hoover gas
		tmp_int  = (int)(IntegralNick/GIER_GRAD_FAKTOR);  // nick angle in deg
		tmp_int2 = (int)(IntegralRoll/GIER_GRAD_FAKTOR);  // roll angle in deg
		tmp_int  = (int16_t)ihypot(tmp_int, tmp_int2); // phytagoras gives effective attitude angle in deg
tmp_int = (tmp_int * Parameter_Hoehe_TiltCompensation) / 100;
		LIMIT_MAX(tmp_int, 60); // limit effective attitude angle
		CosAttitude = c_cos_8192(tmp_int);  // cos of actual attitude
		VarioCharacter = ' ';
		AltitudeSetpointTrimming = 0;
		if(HoehenReglerAktiv && !(FC_StatusFlags & FC_STATUS_EMERGENCY_LANDING))
		{
		// Holger original version
		// start of height control algorithm
		// the height control is only an attenuation of the actual gas stick.
		// I.e. it will work only if the gas stick is higher than the hover gas
		// and the hover height will be allways larger than height setpoint.
		FC_StatusFlags2 |= FC_STATUS2_ALTITUDE_CONTROL;
        if((Parameter_ExtraConfig & CFG2_HEIGHT_LIMIT) || !(Parameter_GlobalConfig & CFG_HOEHEN_SCHALTER))  // Regler wird �ber Schalter gesteuert)
  	      {  // old version
			HCGas = GasMischanteil; // take current stick gas as neutral point for the height control
			HeightTrimming = 0;
			AltitudeSetpointTrimming = 0;
			// set both flags to indicate no vario mode
			FC_StatusFlags |= (FC_STATUS_VARIO_TRIM_UP|FC_STATUS_VARIO_TRIM_DOWN);
          }
		  else
		  {
		// alternative height control
		// PD-Control with respect to hoover point
		// the thrust loss out of horizontal attitude is compensated
		// the setpoint will be fine adjusted with the gas stick position
			if(/*1 || */FC_StatusFlags & FC_STATUS_FLY) // trim setpoint only when flying
			{   // gas stick is above hoover point
				if(StickGas > (StickGasHover + HEIGHT_CONTROL_STICKTHRESHOLD) && !BaroAtUpperLimit)
				{
					if(FC_StatusFlags & FC_STATUS_VARIO_TRIM_DOWN)
					{
						FC_StatusFlags &= ~FC_STATUS_VARIO_TRIM_DOWN;
						SollHoehe = HoehenWertF; // update setpoint to current heigth
					}
					// Limit the maximum Altitude
					if(Parameter_MaximumAltitude && (SollHoehe/100 > Parameter_MaximumAltitude)) 
					 {
					  AltitudeSetpointTrimming = 0;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
					  if(!SpeakHoTT && HoehenWert/95 > Parameter_MaximumAltitude) SpeakHoTT = SPEAK_MAX_ALTITUD;
#endif
					  VarioCharacter = '=';
					 } 
					else 
					{
					if(HeightDeviation > 20) SollHoehe = HoehenWertF; // update setpoint to current heigth 
					FC_StatusFlags |= FC_STATUS_VARIO_TRIM_UP;
					AltitudeSetpointTrimming = abs(StickGas - (StickGasHover + HEIGHT_CONTROL_STICKTHRESHOLD));
					if(LowVoltageLandingActive) AltitudeSetpointTrimming /= 3; // only 33% rising
					VarioCharacter = '+';
					}
					WaypointTrimming = 0;
				} // gas stick is below hoover point
				else if(StickGas < (StickGasHover - HEIGHT_CONTROL_STICKTHRESHOLD) && !BaroAtLowerLimit )     // Minus
				{
					if(FC_StatusFlags & FC_STATUS_VARIO_TRIM_UP || (HeightDeviation < -300))
					{
						FC_StatusFlags &= ~FC_STATUS_VARIO_TRIM_UP;
						SollHoehe = HoehenWertF; // update setpoint to current heigth
					}
					FC_StatusFlags |= FC_STATUS_VARIO_TRIM_DOWN;
					AltitudeSetpointTrimming = -abs(StickGas - (StickGasHover - HEIGHT_CONTROL_STICKTHRESHOLD));
					VarioCharacter = '-';
					WaypointTrimming = 0;
				}
				else // Gas Stick in Hover Range
				{
					VarioCharacter = '=';
                    if(FromNC_AltitudeSpeed && FromNC_AltitudeSetpoint > SollHoehe) // von NC gesteuert -> Steigen
					 {
						FC_StatusFlags |= FC_STATUS_VARIO_TRIM_UP;
						AltitudeSetpointTrimming = FromNC_AltitudeSpeed;
						//HeightTrimming += FromNC_AltitudeSpeed;
						WaypointTrimming = 10;
						VarioCharacter = '^';
						if(FC_StatusFlags & FC_STATUS_VARIO_TRIM_DOWN)  // changed from sinking to rising
						{
							FC_StatusFlags &= ~FC_STATUS_VARIO_TRIM_DOWN;
							SollHoehe = HoehenWertF; // update setpoint to current heigth
						}
					 }
					 else 
                    if(FromNC_AltitudeSpeed && FromNC_AltitudeSetpoint < SollHoehe) // von NC gesteuert -> sinken
					 {
						FC_StatusFlags |= FC_STATUS_VARIO_TRIM_DOWN;
						AltitudeSetpointTrimming = -FromNC_AltitudeSpeed;
						//HeightTrimming -= FromNC_AltitudeSpeed;
						WaypointTrimming = -10;
						VarioCharacter = 'v';
						if(FC_StatusFlags & FC_STATUS_VARIO_TRIM_UP) // changed from rising to sinking
						{
							FC_StatusFlags &= ~FC_STATUS_VARIO_TRIM_UP;
							SollHoehe = HoehenWertF; // update setpoint to current heigth
						}
					 }
					else 
					if(FC_StatusFlags & (FC_STATUS_VARIO_TRIM_UP|FC_STATUS_VARIO_TRIM_DOWN))
					{
						if(!WaypointTrimming) LIMIT_MIN_MAX(SollHoehe, (HoehenWertF-200), (HoehenWertF+200)) // max. 2m Unterschied
						else					WaypointTrimming = 0;
						FC_StatusFlags &= ~(FC_STATUS_VARIO_TRIM_UP|FC_STATUS_VARIO_TRIM_DOWN);
						HeightTrimming = 0;
						if(Parameter_ExtraConfig & CFG2_VARIO_BEEP) beeptime = 500;
						if(!StartTrigger && HoehenWert > 50)
						{
						 StartTrigger = 1;
						}
					}
				}
				// Trim height set point
				HeightTrimming += AltitudeSetpointTrimming;
				if(abs(HeightTrimming) > 500) // bei Waypoint-Flug ist das ca. die 500Hz
				{
					if(WaypointTrimming) 
					 {
					  if(abs(FromNC_AltitudeSetpoint - SollHoehe) < 10) SollHoehe = FromNC_AltitudeSetpoint; 
					  else SollHoehe += WaypointTrimming;
					  } 
					else
					  {
					  if(HeightTrimming > 0)	SollHoehe += EE_Parameter.Hoehe_Verstaerkung / 3;
					  else                    SollHoehe -= EE_Parameter.Hoehe_Verstaerkung / 3;
					  }
					HeightTrimming = 0;
					LIMIT_MIN_MAX(HoehenWertF, (HoehenWert-1024), (HoehenWert+1024)); // max. 10m Unterschied
					LIMIT_MIN_MAX(SollHoehe, (HoehenWertF-1024), (HoehenWertF+1500)); // max. 15m Unterschied
					if(Parameter_ExtraConfig & CFG2_VARIO_BEEP) beeptime = 100;
					//update hoover gas stick value when setpoint is shifted
					if(FromNC_AltitudeSpeed == 0) CalcStickGasHover();
/*
                       if(!EE_Parameter.Hoehe_StickNeutralPoint && FromNC_AltitudeSpeed == 0)
                       {
                           StickGasHover = HoverGas/STICK_GAIN; //rescale back to stick value
                           StickGasHover = (unsigned int)((unsigned int) StickGasHover * UBat) / BattLowVoltageWarning;
                           if(StickGasHover < 70) StickGasHover = 70;
                           else if(StickGasHover > 175) StickGasHover = 175;
                       }
*/
				}
              if(BaroExpandActive) SollHoehe = HoehenWertF; // update setpoint to current altitude if Expanding is active
			} //if FCFlags & MKFCFLAG_FLY
			else
			{
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			 ACC_AltitudeFusion(1); // init the ACC and Altitude fusion
#endif
			 SollHoehe = HoehenWertF - 2000;

 			 if(EE_Parameter.Hoehe_StickNeutralPoint) StickGasHover = EE_Parameter.Hoehe_StickNeutralPoint;
			 else StickGasHover = 127;
			 HoverGas = GasMischanteil;
			 VarioCharacter = '.';
			 }
			HCGas = HoverGas;      // take hover gas (neutral point)
		   }
         if(HoehenWertF > SollHoehe || !(Parameter_ExtraConfig & CFG2_HEIGHT_LIMIT))
		 {
		  if(!ACC_AltitudeControl)
		  {
			// from this point the Heigth Control Algorithm is identical for both versions
			if(BaroExpandActive) // baro range expanding active
			{
				HCGas = HoverGas; // hover while expanding baro adc range
				HeightDeviation = 0;
			} // EOF // baro range expanding active
			else // valid data from air pressure sensor
			{
				// ------------------------- P-Part ----------------------------
				tmp_long = (HoehenWertF - SollHoehe); // positive when too high
				LIMIT_MIN_MAX(tmp_long, -32767L, 32767L);	// avoid overflov when casting to int16_t
				HeightDeviation = (int)(tmp_long); // positive when too high
				tmp_long = (tmp_long * (long)Parameter_Hoehe_P) / 32L; // p-part
				LIMIT_MIN_MAX(tmp_long, -127 * STICK_GAIN, 256 * STICK_GAIN); // more than the full range makes no sense
				GasReduction = tmp_long;
				// ------------------------- D-Part 1: Vario Meter ----------------------------
				tmp_int = VarioMeter / 8;
				LIMIT_MIN_MAX(tmp_int, -127, 128);
				tmp_int = (tmp_int * (long)Parameter_Luftdruck_D) / 4L; // scale to d-gain parameter
				LIMIT_MIN_MAX(tmp_int,-64 * STICK_GAIN, 64 * STICK_GAIN);
				if(FC_StatusFlags & (FC_STATUS_VARIO_TRIM_UP|FC_STATUS_VARIO_TRIM_DOWN)) tmp_int /= 4; // reduce d-part while trimming setpoint
				else
				if(Parameter_ExtraConfig & CFG2_HEIGHT_LIMIT) tmp_int /= 8; // reduce d-part in "Deckel" mode
				GasReduction += tmp_int;
			} // EOF no baro range expanding
			// ------------------------ D-Part 2: ACC-Z Integral  ------------------------
            if(Parameter_Hoehe_ACC_Wirkung)
			 {
			  tmp_long = ((Mess_Integral_Hoch / 128L) * (int32_t) Parameter_Hoehe_ACC_Wirkung) / (128L / STICK_GAIN);
			  LIMIT_MIN_MAX(tmp_long, -32 * STICK_GAIN, 64 * STICK_GAIN);
			  GasReduction += tmp_long;
			 }
/*			// ------------------------ D-Part 3: GpsZ  ----------------------------------
			tmp_int = (Parameter_Hoehe_GPS_Z * (int)FromNaviCtrl_Value.GpsZ)/128L;
            LIMIT_MIN_MAX(tmp_int, -32 * STICK_GAIN, 64 * STICK_GAIN);
			GasReduction += tmp_int;
*/
            GasReduction = (long)((long)GasReduction * HoverGas) / 512; // scale to the gas value

			// ------------------------                  ----------------------------------
			HCGas -= GasReduction;
			// limit deviation from hoover point within the target region
			if(!AltitudeSetpointTrimming && HoverGas > 0) // height setpoint is not changed and hoover gas not zero
			{
			 unsigned int tmp;
 			 tmp = abs(HeightDeviation);
			 if(tmp <= 60)
			 {
			  LIMIT_MIN_MAX(HCGas, HoverGasMin, HoverGasMax); // limit gas around the hoover point
			 }
			 else
			 {
				tmp = (tmp - 60) / 32;
				if(tmp > 15) tmp = 15;
			   if(HeightDeviation > 0)
				{
				tmp = (HoverGasMin * (16 - tmp)) / 16;
				LIMIT_MIN_MAX(HCGas, tmp, HoverGasMax); // limit gas around the hoover point
				}
				else
				{
				tmp = (HoverGasMax * (tmp + 16)) / 16;
				LIMIT_MIN_MAX(HCGas, HoverGasMin, tmp); // limit gas around the hoover point
				}
			  }
			}
			// strech control output by inverse attitude projection 1/cos
            // + 1/cos(angle)  ++++++++++++++++++++++++++
			tmp_long2 = (int32_t)HCGas;
			tmp_long2 *= 8192L;
			tmp_long2 /= CosAttitude;
			HCGas = (int16_t)tmp_long2;
			// update height control gas averaging
			FilterHCGas = (FilterHCGas * (HC_GAS_AVERAGE - 1) + HCGas) / HC_GAS_AVERAGE;
			// limit height control gas pd-control output
			LIMIT_MIN_MAX(FilterHCGas, EE_Parameter.Hoehe_MinGas * STICK_GAIN, (MAX_GAS - 20) * STICK_GAIN);
			// set GasMischanteil to HeightControlGasFilter
            if(Parameter_ExtraConfig & CFG2_HEIGHT_LIMIT)
			{  // old version
				LIMIT_MAX(FilterHCGas, GasMischanteil); // nicht mehr als Gas
				GasMischanteil = FilterHCGas;
			}
			else GasMischanteil = FilterHCGas + (GasMischanteil - HoverGas) / 4; // only in Vario-Mode
		   }
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
          else // ACC-Altitude control
		   {
			// from this point the Heigth Control Algorithm is identical for both versions
			if(BaroExpandActive) // baro range expanding active
			{
				HCGas = HoverGas; // hover while expanding baro adc range
				HeightDeviation = 0;
			} // EOF // baro range expanding active
			else // valid data from air pressure sensor
			{
				// ------------------------- P-Part ----------------------------
				tmp_long = (HoehenWertF - SollHoehe); // positive when too high
				LIMIT_MIN_MAX(tmp_long, -32767L, 32767L);	// avoid overflov when casting to int16_t
				HeightDeviation = (int)(tmp_long); // positive when too high
				tmp_long = (tmp_long * (long)Parameter_Hoehe_P) / 32L; // p-part
				LIMIT_MIN_MAX(tmp_long, -511 * STICK_GAIN, 512 * STICK_GAIN); // more than full range makes sense
				GasReduction = tmp_long;
				// ------------------------ D-Part: ACC-Z Integral  ------------------------
				tmp_long = VarioMeter + (AdWertAccHoch * Parameter_Hoehe_ACC_Wirkung)/256;
				// ------------------------- D-Part: Vario Meter ----------------------------
				if(WaypointTrimming) {
					Variance = AltitudeSetpointTrimming * 8;	
				} else {
					Variance = AltitudeSetpointTrimming * EE_Parameter.Hoehe_Verstaerkung*9/32;
				}
				tmp_long -= (long)Variance;
				tmp_long = (tmp_long * (long)Parameter_Luftdruck_D) / 32; // scale to d-gain parameter
				LIMIT_MIN_MAX(tmp_long,-511 * STICK_GAIN, 512 * STICK_GAIN);
				GasReduction += tmp_long;
			} // EOF no baro range expanding
			HCGas -= GasReduction;
			LIMIT_MIN_MAX(HCGas, HoverGasMin, HoverGasMax); // limits gas around hover point
			// strech control output by inverse attitude projection 1/cos
            // + 1/cos(angle)  ++++++++++++++++++++++++++
			tmp_long2 = (int32_t)HCGas;
			tmp_long2 *= 8192L;
			tmp_long2 /= CosAttitude;
			HCGas = (int16_t)tmp_long2;
			// update height control gas averaging
			FilterHCGas = (FilterHCGas * (HC_GAS_AVERAGE - 1) + HCGas) / HC_GAS_AVERAGE;
			// limit height control gas pd-control output
			int min;
			if(GasIsZeroCnt > 400 || (FC_StatusFlags2 & FC_STATUS2_WAIT_FOR_TAKEOFF)) min = EE_Parameter.Gas_Min; else min = EE_Parameter.Hoehe_MinGas;
			LIMIT_MIN_MAX(FilterHCGas, min * STICK_GAIN, (MAX_GAS - 20) * STICK_GAIN)
			// set GasMischanteil to HeightControlGasFilter
            if(Parameter_ExtraConfig & CFG2_HEIGHT_LIMIT)
			{  // old version
				LIMIT_MAX(FilterHCGas, GasMischanteil); // nicht mehr als Gas
				GasMischanteil = FilterHCGas;
			}
			else GasMischanteil = FilterHCGas;
           } // end of ACC-Altitude control
#endif
		  }
		}// EOF height control active
		else // HC not active
		{
			//update hoover gas stick value when HC is not active
			CalcStickGasHover();
/*			if(!EE_Parameter.Hoehe_StickNeutralPoint)
			{
				StickGasHover = HoverGas/STICK_GAIN; // rescale back to stick value
				StickGasHover = (unsigned int)((unsigned int) StickGasHover * UBat) / BattLowVoltageWarning;
			}
			else StickGasHover = EE_Parameter.Hoehe_StickNeutralPoint;
            LIMIT_MIN_MAX(StickGasHover, 70, 175); // reserve some range for trim up and down
*/		
			FilterHCGas = GasMischanteil;
			// set both flags to indicate no vario mode
			FC_StatusFlags |= (FC_STATUS_VARIO_TRIM_UP|FC_STATUS_VARIO_TRIM_DOWN);
			FC_StatusFlags2 &= ~FC_STATUS2_ALTITUDE_CONTROL;
		}
		// Hover gas estimation by averaging gas control output on small z-velocities
		// this is done only if height contol option is selected in global config and aircraft is flying
		if((FC_StatusFlags & FC_STATUS_FLY))// && !(FC_SatusFlags & FC_STATUS_EMERGENCY_LANDING))
		{
			//if(HoverGasFilter == 0 || StartTrigger == 1)  HoverGasFilter = HOVER_GAS_AVERAGE * (unsigned long)(GasMischanteil); // init estimation
			if(HoverGasFilter == 0 || StartTrigger == 1)  HoverGasFilter = HOVER_GAS_AVERAGE * (unsigned long)(HoverGas); // 0.90f: ge�ndert
			if(StartTrigger == 1) StartTrigger = 2;
				tmp_long2 = (int32_t)GasMischanteil; // take current thrust
				tmp_long2 *= CosAttitude;            // apply attitude projection
				tmp_long2 /= 8192;
				// average vertical projected thrust
			    if(modell_fliegt < 4000) // the first 8 seconds
				{   // reduce the time constant of averaging by factor of 4 to get much faster a stable value
					HoverGasFilter -= HoverGasFilter/(HOVER_GAS_AVERAGE/16L);
					HoverGasFilter += 16L * tmp_long2;
				}
                if(modell_fliegt < 8000) // the first 16 seconds
				{   // reduce the time constant of averaging by factor of 2 to get much faster a stable value
					HoverGasFilter -= HoverGasFilter/(HOVER_GAS_AVERAGE/4L);
					HoverGasFilter += 4L * tmp_long2;
				}
			  else //later
 			  if(abs(VarioMeter) < 100 && abs(HoehenWertF - SollHoehe) < 256) // only on small vertical speed & difference is small (only descending)
				{
					HoverGasFilter -= HoverGasFilter/HOVER_GAS_AVERAGE;
					HoverGasFilter += tmp_long2;
				}
				HoverGas = (int16_t)(HoverGasFilter/HOVER_GAS_AVERAGE);
				if(EE_Parameter.Hoehe_HoverBand)
				{
					int16_t band;
					band = HoverGas / EE_Parameter.Hoehe_HoverBand; // the higher the parameter the smaller the range
					HoverGasMin = HoverGas - band;
					HoverGasMax = HoverGas + band;
				}
				else
				{	// no limit
					HoverGasMin = 0;
					HoverGasMax = 1023;
		 		}
		}
		 else
		  {
		   StartTrigger = 0;
		   HoverGasFilter = 0;
		   HoverGas = 0;
		  }
	}// EOF Parameter_GlobalConfig & CFG_HEIGHT_CONTROL
	else
	{
		// set undefined state to indicate vario off
		FC_StatusFlags |= (FC_STATUS_VARIO_TRIM_UP|FC_STATUS_VARIO_TRIM_DOWN);
	} // EOF no height control

   // Limits the maximum gas in case of "Out of Range emergency landing"
   if(NC_To_FC_Flags & NC_TO_FC_EMERGENCY_LANDING) 
	{
	 if(GasMischanteil/STICK_GAIN > HooverGasEmergencyPercent && HoverGas) GasMischanteil = HooverGasEmergencyPercent * STICK_GAIN;
	 SollHoehe = HoehenWertF; // update setpoint to current heigth
	  beeptime = 15000;
	  BeepMuster = 0x0E00;
	}
    // limit gas to parameter setting
  LIMIT_MIN(GasMischanteil, (MIN_GAS + 10) * STICK_GAIN);
  if(GasMischanteil > (MAX_GAS - 20) * STICK_GAIN) GasMischanteil = (MAX_GAS - 20) * STICK_GAIN;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// all BL-Ctrl connected?
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef REDUNDANT_FC_SLAVE
  if(MissingMotor || Capacity.MinOfMaxPWM < 254 || NC_ErrorCode)      // wait until all BL-Ctrls started and no Errors
  if(modell_fliegt > 1 && modell_fliegt < 50 && GasMischanteil > 0)    // only during start-phase
   {
    modell_fliegt = 1;
	GasMischanteil = (MIN_GAS + 10) * STICK_GAIN;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	if(Capacity.MinOfMaxPWM < 40) SpeakHoTT = SPEAK_ERR_MOTOR;
#endif
   }
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// send SPI pending bytes
 if(BytegapSPI == 0)  SPI_TransmitByte(); 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Mischer und PI-Regler
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  DebugOut.Analog[7] = GasMischanteil;  // achtung: Muss auf [7] bleiben wegen SPI.C
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gier-Anteil
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    GierMischanteil = MesswertGier - sollGier * STICK_GAIN;     // Regler f�r Gier
#define MIN_GIERGAS  (40*STICK_GAIN)  // unter diesem Gaswert trotzdem Gieren
   if(GasMischanteil > MIN_GIERGAS)
    {
     if(GierMischanteil > (GasMischanteil / 2)) GierMischanteil = GasMischanteil / 2;
     if(GierMischanteil < -(GasMischanteil / 2)) GierMischanteil = -(GasMischanteil / 2);
    }
    else
    {
     if(GierMischanteil > (MIN_GIERGAS / 2))  GierMischanteil = MIN_GIERGAS / 2;
     if(GierMischanteil < -(MIN_GIERGAS / 2)) GierMischanteil = -(MIN_GIERGAS / 2);
    }
    tmp_int = MAX_GAS*STICK_GAIN;
    if(GierMischanteil > ((tmp_int - GasMischanteil))) GierMischanteil = ((tmp_int - GasMischanteil));
    if(GierMischanteil < -((tmp_int - GasMischanteil))) GierMischanteil = -((tmp_int - GasMischanteil));

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Nick-Achse
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    DiffNick = MesswertNick - StickNick;	// Differenz bestimmen
    if(IntegralFaktor) SummeNick += IntegralNickMalFaktor - StickNick; // I-Anteil bei Winkelregelung
    else  SummeNick += DiffNick; // I-Anteil bei HH
    if(SummeNick >  (STICK_GAIN * 16000L)) SummeNick =  (STICK_GAIN * 16000L);
    if(SummeNick < -(16000L * STICK_GAIN)) SummeNick = -(16000L * STICK_GAIN);

    if(EE_Parameter.Gyro_Stability <= 8) 	pd_ergebnis_nick = (EE_Parameter.Gyro_Stability * DiffNick) / 8; // PI-Regler f�r Nick
    else 									pd_ergebnis_nick = ((EE_Parameter.Gyro_Stability / 2) * DiffNick) / 4; // �berlauf verhindern
    pd_ergebnis_nick +=  SummeNick / Ki;

    tmp_int = (long)((long)Parameter_DynamicStability * (long)(GasMischanteil + abs(GierMischanteil)/2)) / 64;
    if(pd_ergebnis_nick >  tmp_int) pd_ergebnis_nick =  tmp_int;
    if(pd_ergebnis_nick < -tmp_int) pd_ergebnis_nick = -tmp_int;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Roll-Achse
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	DiffRoll = MesswertRoll - StickRoll;	// Differenz bestimmen
    if(IntegralFaktor) SummeRoll += IntegralRollMalFaktor - StickRoll;// I-Anteil bei Winkelregelung
    else 	         SummeRoll += DiffRoll;  // I-Anteil bei HH
    if(SummeRoll >  (STICK_GAIN * 16000L)) SummeRoll =  (STICK_GAIN * 16000L);
    if(SummeRoll < -(16000L * STICK_GAIN)) SummeRoll = -(16000L * STICK_GAIN);

    if(EE_Parameter.Gyro_Stability <= 8)  	pd_ergebnis_roll = (EE_Parameter.Gyro_Stability * DiffRoll) / 8;	// PI-Regler f�r Roll
	else  									pd_ergebnis_roll = ((EE_Parameter.Gyro_Stability / 2) * DiffRoll) / 4;	// �berlauf verhindern
    pd_ergebnis_roll += SummeRoll / Ki;
	
    tmp_int = (long)((long)Parameter_DynamicStability * (long)(GasMischanteil + abs(GierMischanteil)/2)) / 64;
    if(pd_ergebnis_roll >  tmp_int) pd_ergebnis_roll =  tmp_int;
    if(pd_ergebnis_roll < -tmp_int) pd_ergebnis_roll = -tmp_int;

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// send SPI pending bytes
	if(BytegapSPI == 0)  SPI_TransmitByte(); 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Universal Mixer
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 	for(i=0; i<MAX_MOTORS; i++)
 	{
		signed int tmp_int;
		if(Mixer.Motor[i][0] > 0)
		{
			// Gas
			if(Mixer.Motor[i][0] == 64) tmp_int = GasMischanteil; else tmp_int =  ((long)GasMischanteil * Mixer.Motor[i][0]) / 64L;
			// Nick
			if(Mixer.Motor[i][1] == 64) tmp_int += pd_ergebnis_nick;
			else if(Mixer.Motor[i][1] == -64) tmp_int -= pd_ergebnis_nick;
			else tmp_int += ((long)pd_ergebnis_nick * Mixer.Motor[i][1]) / 64L;
            // Roll
 			if(Mixer.Motor[i][2] == 64) tmp_int += pd_ergebnis_roll;
			else if(Mixer.Motor[i][2] == -64) tmp_int -= pd_ergebnis_roll;
			else tmp_int += ((long)pd_ergebnis_roll * Mixer.Motor[i][2]) / 64L;
            // Gier
 			if(Mixer.Motor[i][3] == 64) tmp_int += GierMischanteil;
			else if(Mixer.Motor[i][3] == -64) tmp_int -= GierMischanteil;
			else tmp_int += ((long)GierMischanteil * Mixer.Motor[i][3]) / 64L;
#ifdef REDUNDANT_FC_SLAVE
			tmp_int = (tmp_motorwert[i] + tmp_int) / 2;     
#else
			if(Motor[i].Version & MOTOR_STATE_FAST_MODE || tmp_int > tmp_motorwert[i]) tmp_int = (tmp_motorwert[i] + tmp_int) / 2;      // Beschleunigen
            else
                         {  // BL-Ctrl 1.0 or 2.0 
                            if(EE_Parameter.MotorSmooth == 0)
                                { 
                                  tmp_int = 2 * tmp_int - tmp_motorwert[i];  // original MotorSmoothing
                                }
							else  // 1 means tmp_int = tmp_int;
                            if(EE_Parameter.MotorSmooth > 1) 
                                {
                                  // If >= 2 then allow >= 50% of the intended step down to rapidly reach the intended value.
                                  tmp_int = tmp_int + ((tmp_motorwert[i] - tmp_int) / EE_Parameter.MotorSmooth);
                                }
                        }
#endif
			LIMIT_MIN_MAX(tmp_int,(int) MIN_GAS * 4,(int) MAX_GAS * 4);
			Motor[i].SetPoint = tmp_int / 4;
			Motor[i].SetPointLowerBits = (tmp_int % 4)<<1; // (3 bits total)
            tmp_motorwert[i] = tmp_int;
		}
		else
		{
			Motor[i].SetPoint = 0;
			Motor[i].SetPointLowerBits = 0;
		}
	}
#if (defined(REDUNDANT_FC_MASTER) || defined(REDUNDANT_FC_SLAVE))
if(Parameter_UserParam6 > 230)  // Motor1-Test
 {
  Motor[0].SetPoint = 0;
  FC_StatusFlags3 |= FC_STATUS3_REDUNDANCE_TEST; 
 } 
 else 
 if(Parameter_UserParam7 <= 150) // I2C-Test
 {
  FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_TEST; 
 } 
#endif
}
//DebugOut.Analog[16]
