// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software Nutzungsbedingungen (english version: see below)
// + der Fa. HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland - nachfolgend Lizenzgeber genannt -
// + Der Lizenzgeber räumt dem Kunden ein nicht-ausschließliches, zeitlich und räumlich* unbeschränktes Recht ein, die im den
// + Mikrocontroller verwendete Firmware für die Hardware Flight-Ctrl, Navi-Ctrl, BL-Ctrl, MK3Mag & PC-Programm MikroKopter-Tool 
// + - nachfolgend Software genannt - nur für private Zwecke zu nutzen.
// + Der Einsatz dieser Software ist nur auf oder mit Produkten des Lizenzgebers zulässig.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die vom Lizenzgeber gelieferte Software ist urheberrechtlich geschützt. Alle Rechte an der Software sowie an sonstigen im
// + Rahmen der Vertragsanbahnung und Vertragsdurchführung überlassenen Unterlagen stehen im Verhältnis der Vertragspartner ausschließlich dem Lizenzgeber zu.
// + Die in der Software enthaltenen Copyright-Vermerke, Markenzeichen, andere Rechtsvorbehalte, Seriennummern sowie
// + sonstige der Programmidentifikation dienenden Merkmale dürfen vom Kunden nicht verändert oder unkenntlich gemacht werden.
// + Der Kunde trifft angemessene Vorkehrungen für den sicheren Einsatz der Software. Er wird die Software gründlich auf deren
// + Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Die Haftung des Lizenzgebers wird - soweit gesetzlich zulässig - begrenzt in Höhe des typischen und vorhersehbaren
// + Schadens. Die gesetzliche Haftung bei Personenschäden und nach dem Produkthaftungsgesetz bleibt unberührt. Dem Lizenzgeber steht jedoch der Einwand 
// + des Mitverschuldens offen.
// + Der Kunde trifft angemessene Vorkehrungen für den Fall, dass die Software ganz oder teilweise nicht ordnungsgemäß arbeitet.
// + Er wird die Software gründlich auf deren Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Der Kunde wird er seine Daten vor Einsatz der Software nach dem Stand der Technik sichern.
// + Der Kunde ist darüber unterrichtet, dass der Lizenzgeber seine Daten im zur Vertragsdurchführung erforderlichen Umfang
// + und auf Grundlage der Datenschutzvorschriften erhebt, speichert, verarbeitet und, sofern notwendig, an Dritte übermittelt.
// + *) Die räumliche Nutzung bezieht sich nur auf den Einsatzort, nicht auf die Reichweite der programmierten Software.
// + #### ENDE DER NUTZUNGSBEDINGUNGEN ####'
// +  Hinweis: Informationen über erweiterte Nutzungsrechte (wie z.B. Nutzung für nicht-private Zwecke) sind auf Anfrage per Email an info(@)hisystems.de verfügbar.
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
unsigned char DisableRcOffBeeping = 0;
unsigned char PlatinenVersion = 10;
unsigned char BattLowVoltageWarning = 94;
unsigned char BattAutoLandingVoltage = 0, BattComingHomeVoltage = 0;
unsigned int FlugMinuten = 0,FlugMinutenGesamt = 0;
unsigned int FlugSekunden = 0;
pVoidFnct_pVoidFnctChar_const_fmt _printf_P;
unsigned char FoundMotors = 0;
unsigned char JetiBeep = 0; // to allow any Morse-Beeping of the Jeti-Box
unsigned char ActiveParamSet = 3;
unsigned char LipoCells = 4;

void PrintLine(void)
{
 printf("\n\r===================================");
}


void CalMk3Mag(void)
{
 static unsigned char stick = 1;
 ChannelAssingment();
 if(ChannelNick > -20) stick = 0;
 if((ChannelNick < -70) && !stick)
  {
   stick = 1;
   WinkelOut.CalcState++;
   if(WinkelOut.CalcState > 4)
    {
//     WinkelOut.CalcState = 0; // in Uart.c
     beeptime = 1000;
    }
   else Piep(WinkelOut.CalcState,150);
  }
}


void LipoDetection(unsigned char print)
{
unsigned int warning;
	#define MAX_CELL_VOLTAGE 43 // max cell voltage for LiPO
	if(print) 
	 {
	  printf("\n\rBatt:");
	  LipoCells = 1 + UBat / MAX_CELL_VOLTAGE; 
	  if(LipoCells > 6) LipoCells = 6;
	 } 

	if(EE_Parameter.UnterspannungsWarnung < 50) 
	{
		warning = LipoCells * EE_Parameter.UnterspannungsWarnung;
		if(print)
		{
			Piep(LipoCells, 200);
			printf(" %d Cells ", LipoCells);
		}
	}
	else warning = EE_Parameter.UnterspannungsWarnung;
    if(warning > 255) warning = 255; BattLowVoltageWarning = warning;
	// automatische Zellenerkennung
	if(EE_Parameter.AutoLandingVoltage < 50) warning = LipoCells * EE_Parameter.AutoLandingVoltage; else warning = EE_Parameter.AutoLandingVoltage;
	if(warning > 255) warning = 255; BattAutoLandingVoltage = warning;
	
	if(EE_Parameter.ComingHomeVoltage < 50)  warning = LipoCells * EE_Parameter.ComingHomeVoltage; else warning = EE_Parameter.ComingHomeVoltage;
	if(warning > 255) warning = 255; BattComingHomeVoltage = warning;

	if(BattAutoLandingVoltage > BattLowVoltageWarning) BattAutoLandingVoltage = BattLowVoltageWarning - 1; 
	if(BattComingHomeVoltage  >= BattLowVoltageWarning) BattComingHomeVoltage  = BattLowVoltageWarning - 1; 
	if(BattAutoLandingVoltage >= BattComingHomeVoltage && EE_Parameter.ComingHomeVoltage) BattAutoLandingVoltage = BattComingHomeVoltage - 1; 
	
	if(print) 
	 {
	  printf(" Low warning: %d.%dV",BattLowVoltageWarning/10,BattLowVoltageWarning%10);
	  if(BattComingHomeVoltage) printf("  Auto-CH: %d.%dV",BattComingHomeVoltage/10,BattComingHomeVoltage%10);
	  if(BattAutoLandingVoltage) printf("  Autolanding: %d.%dV",BattAutoLandingVoltage/10,BattAutoLandingVoltage%10);
	 } 

}

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
void LoadStoreSingleWP(void)
{
// +++++++++++++++++++++++++++++++++++++++++++
// + Load/Store one single point
// +++++++++++++++++++++++++++++++++++++++++++
static unsigned char switch_hyterese = 0, hyterese = 1, wp_tmp_s = 0, wp_tmp_l = 0;
static int delay;

 if(PPM_in[EE_Parameter.SingleWpControlChannel] > 50)  // Switch Up -> load
 {
    if(switch_hyterese == 1 || switch_hyterese == 3) 
	 {
	  ToNC_Load_SingePoint = 1;
	  switch_hyterese = 2;
	  SpeakHoTT = SPEAK_NEXT_WP;
	  Show_Load_Time = 5;
	  Show_Load_Value = 1;
	  wp_tmp_l = 1;
	 } 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Bedienung per Taster am Sender
  if(PPM_in[EE_Parameter.MenuKeyChannel] > 50)  // 
   {
    hyterese = 2;
    if(CheckDelay(delay)) { wp_tmp_l = 0; hyterese = 1;}
   }
  else
  if(PPM_in[EE_Parameter.MenuKeyChannel] < -50)  
   {
	delay = SetDelay(2500);
	if(hyterese == 2 && (wp_tmp_l < NaviData_MaxWpListIndex))
	 {
	  wp_tmp_l++;
	  ToNC_Load_SingePoint = wp_tmp_l;
	  Show_Load_Time = 5;
	  Show_Load_Value = wp_tmp_l;
	  SpeakHoTT = SPEAK_NEXT_WP;
	 }
    hyterese = 0; 
   }
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 }
 else
 if(PPM_in[EE_Parameter.SingleWpControlChannel] < -50) // Switch Down -> store
 {
    if(switch_hyterese == 1 || switch_hyterese == 2) 
	 {
	  ToNC_Store_SingePoint = 1;
	  switch_hyterese = 3;
	  SpeakHoTT = SPEAK_MIKROKOPTER;
	  Show_Store_Time = 5;
	  Show_Store_Value = 1;
	  wp_tmp_s = 1;
	 } 
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Bedienung per Taster am Sender
  if(PPM_in[EE_Parameter.MenuKeyChannel] > 50)  // 
   {
    hyterese = 2;
    if(CheckDelay(delay)) { wp_tmp_s = 0; hyterese = 1;}
   }
  else
  if(PPM_in[EE_Parameter.MenuKeyChannel] < -50)  
   {
	delay = SetDelay(2500);
	if(hyterese == 2 && (wp_tmp_s < NaviData_MaxWpListIndex))
	 {
	  wp_tmp_s++;
	  ToNC_Store_SingePoint = wp_tmp_s;
	  Show_Store_Time = 5;
	  Show_Store_Value = wp_tmp_s;
	  SpeakHoTT = SPEAK_MIKROKOPTER;
	 }
    hyterese = 0; 
   }
 }
 else  // Middle
 {
    switch_hyterese = 1;
 }
}
// +++++++++++++++++++++++++++++++++++++++++++
#endif
//############################################################################
//Hauptprogramm
int main (void)
//############################################################################
{
	unsigned int timer,i,timer2 = 0, timerPolling;
	unsigned char update_spi = 1;
    DDRB  = 0x00;
    PORTB = 0x00; 
    DDRD  = 0x0A; // UART & J3 J4 J5
	PORTD = 0x5F; // PPM-Input & UART
    for(timer = 0; timer < 1000; timer++); // verzögern
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	unsigned char AccZ_ErrorCnt = 0;
    if(PINB & 0x02) 
	 {
	  if(PIND & 0x10) PlatinenVersion = 21; // No Bridge from J4 to GND
	  else { PlatinenVersion = 22; ACC_AltitudeControl = 1;};
	 }
     else          
     {
	  PlatinenVersion = 25; 
	  ACC_AltitudeControl = 1;
	  J4Low;
  	 }  
#else
	if(PINB & 0x01)
     {
      if(PINB & 0x02) PlatinenVersion = 13;
       else           PlatinenVersion = 11;
     }
    else
     {
      if(PINB & 0x02) PlatinenVersion = 20;
       else           
	    {
		 PlatinenVersion = 10;
		 DDRD  = 0x3E; // Speaker & TXD & J3 J4 J5
  	     PORTD = 0x47; // 
		} 
     }
#endif

    DDRC  = 0x81; // I2C, Spaker
    DDRC  |=0x40; // HEF4017 Reset
    PORTC = 0xff; // Pullup SDA
    DDRB  = 0x1B; // LEDs und Druckoffset
    PORTB = 0x01; // LED_Rot

    HEF4017Reset_ON;
    MCUSR &=~(1<<WDRF);
    WDTCSR |= (1<<WDCE)|(1<<WDE);
    WDTCSR = 0;

    beeptime = 2500;
	StickGier = 0; StickRoll = 0; StickNick = 0;
    if(PlatinenVersion >= 20)  GIER_GRAD_FAKTOR = 1220; else GIER_GRAD_FAKTOR = 1291; // unterschiedlich für ME und ENC
    ROT_OFF;
    GRN_ON;

    Timer_Init();
	TIMER2_Init();
	UART_Init();
    rc_sum_init();
   	ADC_Init();
	I2C_Init(1);
	SPI_MasterInit();
	Capacity_Init();
	LIBFC_Init(LIB_FC_COMPATIBLE);
	GRN_ON;
    sei();
	ParamSet_Init();
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(PlatinenVersion < 20) 
	{
	    wdt_enable(WDTO_250MS); // Reset-Commando
		while(1) printf("\n\rOld FC Hardware not supported by this Firmware!");
	}
#ifndef REDUNDANT_FC_SLAVE
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Check connected BL-Ctrls
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Check connected BL-Ctrls
	BLFlags |= BLFLAG_READ_VERSION;
	motor_read = 0;  // read the first I2C-Data
	for(i=0; i < 500; i++)
	{
 	 SendMotorData();
	 timer = SetDelay(5);
	 while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer
    }
    printf("\n\rFound BL-Ctrl: ");
//    timer = SetDelay(1000);
	for(i=0; i < MAX_MOTORS; i++)
	{
//		SendMotorData();
//		while(!(BLFlags & BLFLAG_TX_COMPLETE)  && !CheckDelay(timer)); //wait for complete transfer
		if(Mixer.Motor[i][0] > 0) // wait max 4 sec for the BL-Ctrls to wake up
		{
			while(!CheckDelay(timer) && !(Motor[i].State & MOTOR_STATE_PRESENT_MASK) )
			{
				if((BLFlags & BLFLAG_TX_COMPLETE)) SendMotorData();
				//while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer
			}
		}
		if(Motor[i].State & MOTOR_STATE_PRESENT_MASK)
		{
		    unsigned char vers;
			printf("%d",(i+1)%10);
			FoundMotors++;
			vers = Motor[i].VersionMajor * 100 + Motor[i].VersionMinor; // creates 104 from 1.04
			if(vers && VersionInfo.BL_Firmware > vers) VersionInfo.BL_Firmware = vers;
//if(Motor[i].Version & MOTOR_STATE_FAST_MODE) printf("(fast)");
//if(Motor[i].Version & MOTOR_STATE_NEW_PROTOCOL_MASK) printf("(new)");
//printf(":V%03d\n\r",vers);
		}
	}
	for(i=0; i < MAX_MOTORS; i++)
	{
		if(!(Motor[i].State & MOTOR_STATE_PRESENT_MASK) && Mixer.Motor[i][0] > 0)
		{
			printf("\n\r\n\r!! MISSING BL-CTRL: %d !!",i+1);
			ServoActive = 2; // just in case the FC would be used as camera-stabilizer
		}
		Motor[i].State &= ~MOTOR_STATE_ERROR_MASK; // clear error counter
	}
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
   if(VersionInfo.BL_Firmware != 255) 
    {
	 printf("\n\rBL-Firmware %d.%02d",VersionInfo.BL_Firmware/100,VersionInfo.BL_Firmware%100);
	 if(VersionInfo.BL_Firmware >= 100 && VersionInfo.BL_Firmware <= 102) printf("<-- warning old Version!");
	} 
#endif

   PrintLine();// ("\n\r===================================");
    if(RequiredMotors < FoundMotors) VersionInfo.HardwareError[1] |= FC_ERROR1_MIXER;
	if(RequiredMotors > 8) Max_I2C_Packets = 8; else Max_I2C_Packets = RequiredMotors;
#else
 printf("\n\r\n\r--> REDUNDANT SLAVE <---\n\r");
#endif

#ifdef REDUNDANT_FC_MASTER
 printf("\n\r\n\r--> REDUNDANT MASTER <---\n\r");
#endif

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Calibrating altitude sensor
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//if(EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG)
	{
		printf("\n\rCalibrating pressure sensor..");
		timer = SetDelay(1000);
		SucheLuftruckOffset();
		while (!CheckDelay(timer));
		printf("OK\n\r");
	}

#ifdef REDUNDANT_FC_SLAVE
VersionInfo.HardwareError[0] = 0;
VersionInfo.HardwareError[1] = 0;
#endif

	SetNeutral(0);

	ROT_OFF;

    beeptime = 2000;

	FlugMinuten = (unsigned int)GetParamByte(PID_FLIGHT_MINUTES) * 256 + (unsigned int)GetParamByte(PID_FLIGHT_MINUTES + 1);
	FlugMinutenGesamt = (unsigned int)GetParamByte(PID_FLIGHT_MINUTES_TOTAL) * 256 + (unsigned int)GetParamByte(PID_FLIGHT_MINUTES_TOTAL + 1);

	if((FlugMinutenGesamt == 0xFFFF) || (FlugMinuten == 0xFFFF))
	{
		FlugMinuten = 0;
		FlugMinutenGesamt = 0;
	}
    printf("\n\rFlight-time %u min Total:%u min", FlugMinuten, FlugMinutenGesamt);
    LcdClear();
    I2CTimeout = 5000;
    WinkelOut.Orientation = 1;
    LipoDetection(1);
	LIBFC_ReceiverInit(EE_Parameter.Receiver);
    PrintLine();// ("\n\r===================================");
	//SpektrumBinding();
    timer = SetDelay(2000);
	timerPolling = SetDelay(250);

	Debug(ANSI_CLEAR "FC-Start!\n\rFlugzeit: %d min", FlugMinutenGesamt);  	// Note: this won't waste flash memory, if #DEBUG is not active
//printf("\n\rEE_Parameter size:%i\n\r", PARAMSET_STRUCT_LEN);

    DebugOut.Status[0] = 0x01 | 0x02;
	JetiBeep = 0;
    if(EE_Parameter.ExtraConfig & CFG_NO_RCOFF_BEEPING)	  DisableRcOffBeeping = 1;
	ReadBlSize = 3; // don't read the version any more
#ifdef REDUNDANT_FC_SLAVE
	timer = SetDelay(2500);
	while(!CheckDelay(timer));
	printf("\n\rStart\n\r");
#endif
	while(1)
	{
	EEAR = EE_DUMMY;  // Set the EEPROM Address pointer to an unused space
	if(ReceiverUpdateModeActive) while (1) PORTC &= ~(1<<7); // Beeper off
	if(UpdateMotor && AdReady)      // ReglerIntervall
            {
cli();
			UpdateMotor--;    
sei();
            if(WinkelOut.CalcState) CalMk3Mag();
            else  MotorRegler();
			SendMotorData();
            ROT_OFF;
            if(SenderOkay)  { SenderOkay--; /*VersionInfo.HardwareError[1] &= ~FC_ERROR1_PPM;*/ }
			else
			{
				TIMSK1 |= _BV(ICIE1); // enable PPM-Input
				PPM_in[0] = 0; // set RSSI to zero on data timeout
				VersionInfo.HardwareError[1] |= FC_ERROR1_PPM;
				// Now clear the channel values - they would be wrong
				PPM_diff[EE_Parameter.Kanalbelegung[K_NICK]] = 0;
				PPM_diff[EE_Parameter.Kanalbelegung[K_ROLL]] = 0;
				PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] = 0;
				PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]] = 0;
				PPM_in[EE_Parameter.Kanalbelegung[K_GIER]] = 0;
				PPM_in[EE_Parameter.Kanalbelegung[K_GAS]] = 0;
				ChannelNick = 0;
				ChannelRoll = 0;
				ChannelYaw = 0;
				ChannelGas = 0;
			}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//if(HoehenReglerAktiv && NaviDataOkay && SenderOkay < 160 && SenderOkay > 10 && FromNaviCtrl_Value.SerialDataOkay > 220) SenderOkay = 160;
//if(HoehenReglerAktiv && NaviDataOkay && SenderOkay < 101 && SenderOkay > 10 && FromNaviCtrl_Value.SerialDataOkay > 1) SenderOkay = 101;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            if(!--I2CTimeout || MissingMotor)
                {
                  if(!I2CTimeout)
				   {
				    I2C_Reset();
                    I2CTimeout = 5;
					DebugOut.Analog[28]++; // I2C-Error
					VersionInfo.HardwareError[1] |= FC_ERROR1_I2C;
					DebugOut.Status[1] |= 0x02; // BL-Error-Status
  				   }
                  if((BeepMuster == 0xffff) && MotorenEin)
                   {
                    beeptime = 25000;
                    BeepMuster = 0x0080;
                   }
                }
            else
                {
                 ROT_OFF;
                }
   		  LIBFC_Polling();

          if(!UpdateMotor)
		  { 
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
		   if(NewSBusData) ProcessSBus();  
		   else
		   if(NewMlinkData) ProcessMlinkData();
           else
#endif
		   {
			if(BytegapSPI == 0)  SPI_TransmitByte(); 
			if(CalculateServoSignals) CalculateServo();
			DatenUebertragung();
			BearbeiteRxDaten();
			if(CheckDelay(timer))
			{
				static unsigned char second;
				timer += 20; // 20 ms interval
				CalcNickServoValue();
				// ++++++++++++++++++++++++++++
				// + New direction setpoint from NC
				if(NC_CompassSetpoint != -1)
				{
				   int diff;
				   if(!NeueKompassRichtungMerken && (KompassSollWert != NC_CompassSetpoint) && (CareFree || NCForcesNewDirection)) 
				    {
				     diff = ((540 + (KompassSollWert - NC_CompassSetpoint)) % 360) - 180;
					 if(diff > 2) diff = 2;    // max. 2° in 20ms = 100°/sec
					 else
					 if(diff < -2) diff = -2;
					 KompassSollWert -= diff;
					}
					else 
					 {
					  NC_CompassSetpoint = -1;
					  NCForcesNewDirection = 0; // allows Yawing without CareFree (Yawing at Coming Home)
					 } 
				 }
				// ++++++++++++++++++++++++++++
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				if(EE_Parameter.Receiver == RECEIVER_HOTT) HoTT_Menu();
				else 
				if(EE_Parameter.Receiver == RECEIVER_JETI) BuildJeti_Vario();
				// ++++++++++++++++++++++++++++
				// + check the ACC-Z range
				if(ACC_AltitudeControl && ((Aktuell_az < 300) || (DebugOut.Analog[7] < (128 * 4) && Aktuell_az > 850))) // DebugOut.Analog[7] = GasMischanteil
				 {
				  if(++AccZ_ErrorCnt > 50) 
				   {
				    if(MotorenEin) VersionInfo.HardwareError[0] |= FC_ERROR0_ACC_TOP; 
					else CalibrationDone = 0;
				   }
 				 }
				 else AccZ_ErrorCnt = 0;
				// ++++++++++++++++++++++++++++
#endif
				if(MissingMotor || Capacity.MinOfMaxPWM < 30)
				 {
				  if(MissingMotor) VersionInfo.HardwareError[1] |= FC_ERROR1_BL_MISSING;
				  DebugOut.Status[1] |= 0x02; // BL-Error-Status
				 }
				 else
				 {
				   if(!beeptime) 
				    {
				     if(I2CTimeout > 6) DebugOut.Status[1] &= ~0x02; // BL-Error-Status
					} 
				 }
				if(DisableRcOffBeeping) if(SenderOkay > 150) { DisableRcOffBeeping = 0; beeptime = 5000;};
				if(PcZugriff) PcZugriff--;
				else
				{
					if(!SenderOkay)
					{
					  if(BeepMuster == 0xffff && DisableRcOffBeeping != 2)  
					  {
						  beeptime = 15000;
						  BeepMuster = 0x0c00;
						  if(DisableRcOffBeeping) DisableRcOffBeeping = 2;
					  }
					} 
				}
				if(NaviDataOkay > 200)
				{
					NaviDataOkay--;
					VersionInfo.HardwareError[1] &= ~FC_ERROR1_SPI_RX;
					VersionInfo.Flags |= FC_VERSION_FLAG_NC_PRESENT;
				}
				else
				{
					if(NC_Version.Compatible)
					 {
					   VersionInfo.HardwareError[1] |= FC_ERROR1_SPI_RX;
					   NC_ErrorCode = 9; // "ERR: no NC communication"
                       if(BeepMuster == 0xffff && MotorenEin)
						{
							beeptime = 15000;
							BeepMuster = 0xA800;
						}
					 }
					GPS_Nick = 0;
					GPS_Roll = 0;
					GPS_Aid_StickMultiplikator = 0;
					GPSInfo.Flags = 0;
					FromNaviCtrl_Value.Kalman_K = -1;
					FromNaviCtrl.AccErrorN = 0;
					FromNaviCtrl.AccErrorR = 0;
                    FromNaviCtrl.CompassValue = -1;
					FromNC_AltitudeSpeed = 0;
					FromNC_AltitudeSetpoint = 0;
					VersionInfo.Flags &= ~FC_VERSION_FLAG_NC_PRESENT;
					NC_To_FC_Flags = 0;
                    NaviDataOkay = 0;
				}
			   if(UBat <= BattLowVoltageWarning)
				{
					FC_StatusFlags |= FC_STATUS_LOWBAT;
					if(BeepMuster == 0xffff && UBat > 10) // Do not beep, if the voltage reading is below 1V (Supplied via MKUSB)
					{
						beeptime = 6000;
						BeepMuster = 0x0300;
					}
				}
				// +++++++++++++++++++++++++++++++++
				if(ExternalControlTimeout) 
				 {
				  ExternalControlTimeout--;
				  if(ExternalControlTimeout == 1) 
				    { 
					  ExternalControl.Config = 0; 
					  beeptime = 2000;
					}  
				 } 
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
// +++++++++++++++++++++++++++++++++++++++++++
// + Load/Store one single point
// +++++++++++++++++++++++++++++++++++++++++++
				if(EE_Parameter.SingleWpControlChannel) LoadStoreSingleWP();
// +++++++++++++++++++++++++++++++++++++++++++
#endif				   
#ifdef NO_RECEIVER
 PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] = 0; PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]] = 0; PPM_in[EE_Parameter.Kanalbelegung[K_GIER]] = 0; PPM_in[EE_Parameter.Kanalbelegung[K_GAS]] = 0;
 PPM_in[EE_Parameter.HoeheChannel] = (unsigned char) 200;
 PPM_in[EE_Parameter.NaviGpsModeChannel] = (unsigned char) 200;
 PPM_in[EE_Parameter.CareFreeChannel] = (unsigned char) 200;
 SenderOkay = 180;
 MotorenEin = 0;
#endif

				// +++++++++++++++++++++++++++++++++
				// Sekundentakt
                if(++second == 49)
				 {
				   second = 0;

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				   if(ShowSettingNameTime) ShowSettingNameTime--;
				   if(Show_Load_Time) Show_Load_Time--;
				   if(Show_Store_Time) Show_Store_Time--;
#endif				   
				   if(NC_To_FC_Flags & NC_TO_FC_FAILSAFE_LANDING)  ServoFailsafeActive = SERVO_FS_TIME;
				   else
				   if(ServoFailsafeActive) ServoFailsafeActive--;

				   if(FC_StatusFlags & FC_STATUS_FLY) FlugSekunden++;
				   else timer2 = 1450; // 0,5 Minuten aufrunden
				   if(modell_fliegt < 1024)
				    {
					 if(StartLuftdruck < Luftdruck) StartLuftdruck += 5; 
					 else
					 if(StartLuftdruck > Luftdruck) StartLuftdruck -= 5; 
					 FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_ERROR; 
					} 
					else 
					{
#ifdef REDUNDANT_FC_MASTER
					  if(!(FC_StatusFlags3 & FC_STATUS3_REDUNDANCE_AKTIVE)) FC_StatusFlags3 |= FC_STATUS3_REDUNDANCE_ERROR; 
					  else FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_ERROR; 
#endif
					}
			     if(UBat > BattLowVoltageWarning + 1) FC_StatusFlags &= ~FC_STATUS_LOWBAT;
				 }
				// +++++++++++++++++++++++++++++++++
				if(++timer2 == 2930)  // eine Minute
				 {
				   timer2 = 0;
               	   FlugMinuten++;
	               FlugMinutenGesamt++;
                   SetParamByte(PID_FLIGHT_MINUTES,FlugMinuten / 256);
                   SetParamByte(PID_FLIGHT_MINUTES+1,FlugMinuten % 256);
                   SetParamByte(PID_FLIGHT_MINUTES_TOTAL,FlugMinutenGesamt / 256);
                   SetParamByte(PID_FLIGHT_MINUTES_TOTAL+1,FlugMinutenGesamt % 256);
				   timer = SetDelay(20); // falls "timer += 20;" mal nicht geht
	  		     }
			}
           LED_Update();
           Capacity_Update();
           } 
          } //else DebugOut.Analog[18]++;
		  if(update_spi) update_spi--;
		 } // 500Hz
	 if(update_spi == 0) // 41Hz
	  { 
	    if(SPI_StartTransmitPacket()) update_spi = 12; 
		else 
		if(BytegapSPI == 0)  SPI_TransmitByte();  
	   }  
	 else if(BytegapSPI == 0)  SPI_TransmitByte(); 
    }
}
//DebugOut.Analog[] 

