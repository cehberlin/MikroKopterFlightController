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
unsigned int FlugMinuten = 0,FlugMinutenGesamt = 0;
unsigned int FlugSekunden = 0;
pVoidFnct_pVoidFnctChar_const_fmt _printf_P;
unsigned char FoundMotors = 0;
unsigned char JetiBeep = 0; // to allow any Morse-Beeping of the Jeti-Box
unsigned char ActiveParamSet = 3;

void PrintLine(void)
{
 printf("\n\r===================================");
}


void CalMk3Mag(void)
{
 static unsigned char stick = 1;
 if(PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] > -20) stick = 0;
 if((PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] < -70) && !stick)
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
	#define MAX_CELL_VOLTAGE 43 // max cell volatage for LiPO
	unsigned int timer, cells;
	if(print) printf("\n\rBatt:");
	if(EE_Parameter.UnterspannungsWarnung < 50) // automatische Zellenerkennung
	{
		timer = SetDelay(500);
		if(print) while (!CheckDelay(timer));
		// up to 6s LiPo, less than 2s is technical impossible
		for(cells = 2; cells < 7; cells++)
		{
			if(UBat < cells * MAX_CELL_VOLTAGE) break;
		}

		BattLowVoltageWarning = cells * EE_Parameter.UnterspannungsWarnung;
		if(print)
		{
			Piep(cells, 200);
			printf(" %d Cells ", cells);
		}
	}
	else BattLowVoltageWarning = EE_Parameter.UnterspannungsWarnung;
	if(print) printf(" Low warning: %d.%d",BattLowVoltageWarning/10,BattLowVoltageWarning%10);
}

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
	  PlatinenVersion = 23; ACC_AltitudeControl = 1;
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
	StickGier = 0; PPM_in[K_GAS] = 0; StickRoll = 0; StickNick = 0;
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
// + Check connected BL-Ctrls
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Check connected BL-Ctrls
	BLFlags |= BLFLAG_READ_VERSION;
	motor_read = 0;  // read the first I2C-Data
	SendMotorData();
	timer = SetDelay(500);
	while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer

    printf("\n\rFound BL-Ctrl: ");
    timer = SetDelay(4000);
	for(i=0; i < MAX_MOTORS; i++)
	{
		SendMotorData();
		while(!(BLFlags & BLFLAG_TX_COMPLETE)  && !CheckDelay(timer)); //wait for complete transfer
		if(Mixer.Motor[i][0] > 0) // wait max 4 sec for the BL-Ctrls to wake up
		{
			while(!CheckDelay(timer) && !(Motor[i].State & MOTOR_STATE_PRESENT_MASK) )
			{
				SendMotorData();
				while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer
			}
		}
		if(Motor[i].State & MOTOR_STATE_PRESENT_MASK)
		{
			printf("%d",i+1);
			FoundMotors++;
//			if(Motor[i].Version & MOTOR_STATE_NEW_PROTOCOL_MASK) printf("(new) ");
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
   PrintLine();// ("\n\r===================================");


    if(RequiredMotors < FoundMotors) VersionInfo.HardwareError[1] |= FC_ERROR1_MIXER;

	//if(EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG)
	{
		printf("\n\rCalibrating pressure sensor..");
		timer = SetDelay(1000);
		SucheLuftruckOffset();
		while (!CheckDelay(timer));
		printf("OK\n\r");
	}

	SetNeutral(0);

	ROT_OFF;

    beeptime = 2000;
    ExternControl.Digital[0] = 0x55;


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
	EEAR = EE_DUMMY;  // Set the EEPROM Address pointer to an unused space
	while(1)
	{
	if(ReceiverUpdateModeActive) while (1) PORTC &= ~(1<<7); // Beeper off
//GRN_ON;
	if(UpdateMotor && AdReady)      // ReglerIntervall
            {
//GRN_OFF;
			UpdateMotor=0;    
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
#endif
		   {
			if(CalculateServoSignals) CalculateServo();
			DatenUebertragung();
			BearbeiteRxDaten();
			if(CheckDelay(timer))
			{
				static unsigned char second;
				timer += 20; // 20 ms interval
				CalcNickServoValue();
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
				if(MissingMotor)
				 {
				  VersionInfo.HardwareError[1] |= FC_ERROR1_BL_MISSING;
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
					ExternControl.Config = 0;
					ExternStickNick = 0;
					ExternStickRoll = 0;
					ExternStickGier = 0;
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
                    NaviDataOkay = 0;
				}
			   if(UBat < BattLowVoltageWarning)
				{
					FC_StatusFlags |= FC_STATUS_LOWBAT;
					if(BeepMuster == 0xffff)
					{
						beeptime = 6000;
						BeepMuster = 0x0300;
					}
				}
				else if(!beeptime) FC_StatusFlags &= ~FC_STATUS_LOWBAT;
				SendSPI = SPI_BYTEGAP;
				EEAR = EE_DUMMY;  // Set the EEPROM Address pointer to an unused space
				// +++++++++++++++++++++++++++++++++
				// Sekundentakt
                if(++second == 49)
				 {
				   second = 0;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
				   if(ShowSettingNameTime) ShowSettingNameTime--;
#endif				   
				   if(FC_StatusFlags & FC_STATUS_FLY) FlugSekunden++;
				   else timer2 = 1450; // 0,5 Minuten aufrunden
				   if(modell_fliegt < 1024)
				    {
					 if(StartLuftdruck < Luftdruck) StartLuftdruck += 5; 
					 else
					 if(StartLuftdruck > Luftdruck) StartLuftdruck -= 5; 
					} 
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
           } //else DebugOut.Analog[26]++;
          }
		  if(update_spi) update_spi--;
		 } // 500Hz
	 if(update_spi == 0) { SPI_StartTransmitPacket(); update_spi = 12;}  // 41Hz
	 else if(!SendSPI) { SPI_TransmitByte(); }
    }
}
//DebugOut.Analog[16]

