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
#define MULTIPLYER 4

volatile unsigned int CountMilliseconds = 0;
volatile unsigned int tim_main;
volatile unsigned char UpdateMotor = 0;
volatile unsigned int cntKompass = 0;
volatile unsigned int beeptime = 0;
volatile unsigned char BytegapSPI = 0, ServoActive = 0, CalculateServoSignals = 1;
unsigned char JustMK3MagConnected = 0;
uint16_t RemainingPulse = 0;
volatile int16_t ServoNickOffset = (255 / 2) * MULTIPLYER * 16; // initial value near center positon
volatile int16_t ServoRollOffset = (255 / 2) * MULTIPLYER * 16; // initial value near center positon

unsigned int BeepMuster = 0xffff;
signed int NickServoValue = 128 * MULTIPLYER * 16;

volatile int16_t	ServoNickValue = 0;
volatile int16_t	ServoRollValue = 0;


enum {
  STOP             = 0,
  CK               = 1,
  CK8              = 2,
  CK64             = 3,
  CK256            = 4,
  CK1024           = 5,
  T0_FALLING_EDGE  = 6,
  T0_RISING_EDGE   = 7
};


ISR(TIMER0_OVF_vect)    // 9,7kHz
{
   static unsigned char cnt_1ms = 1,cnt = 0;
   unsigned char pieper_ein = 0;
   if(BytegapSPI) BytegapSPI--;
   if(SpektrumTimer) SpektrumTimer--;
   if(!cnt--)
    {
     cnt = 9;
     CountMilliseconds++;
     cnt_1ms++;
     cnt_1ms %= 2;

     if(!cnt_1ms) if(UpdateMotor < 4) UpdateMotor++;
	 if(!(PINC & 0x10)) JustMK3MagConnected = 1;

     if(beeptime)
        {
        if(beeptime > 10) beeptime -= 10; else beeptime = 0;
        if(beeptime & BeepMuster)
         {
          pieper_ein = 1;
         }
         else pieper_ein = 0;
        }
     else
      {
       pieper_ein = 0;
       BeepMuster = 0xffff;
      }
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
     if(pieper_ein) PORTC |= (1<<7); // Speaker an PORTC.7
     else           PORTC &= ~(1<<7);
#else
     if(pieper_ein)
        {
//          if(PlatinenVersion == 10) PORTD |= (1<<2); // Speaker an PORTD.2
//          else                      
		  PORTC |= (1<<7); // Speaker an PORTC.7
        }
     else
        {
//         if(PlatinenVersion == 10) PORTD &= ~(1<<2);
//         else                      
		 PORTC &= ~(1<<7);
        }
#endif
	}
 if(JustMK3MagConnected && !NaviDataOkay && Parameter_GlobalConfig & CFG_KOMPASS_AKTIV)
 {
  if(PINC & 0x10)
   {
	if(++cntKompass > 1000) JustMK3MagConnected = 0;
   }
  else
   {
    if((cntKompass) && (cntKompass < 362))
    {
     cntKompass += cntKompass / 41;
     if(cntKompass > 10) KompassValue = cntKompass - 10; else KompassValue = 0;
    }
    cntKompass = 0;
   }
 }
}


// -----------------------------------------------------------------------
unsigned int SetDelay(unsigned int t)
{
//  TIMSK0 &= ~_BV(TOIE0);
  return(CountMilliseconds + t + 1);
//  TIMSK0 |= _BV(TOIE0);
}

// -----------------------------------------------------------------------
char CheckDelay(unsigned int t)
{
//  TIMSK0 &= ~_BV(TOIE0);
  return(((t - CountMilliseconds) & 0x8000) >> 9);
//  TIMSK0 |= _BV(TOIE0);
}

// -----------------------------------------------------------------------
void Delay_ms(unsigned int w)
{
 unsigned int akt;
 akt = SetDelay(w);
 while (!CheckDelay(akt));
}

void Delay_ms_Mess(unsigned int w)
{
 unsigned int akt;
 akt = SetDelay(w);
 while (!CheckDelay(akt)) if(AdReady) {AdReady = 0; ANALOG_ON;}
}

/*****************************************************/
/*              Initialize Timer 2                   */
/*****************************************************/
// The timer 2 is used to generate the PWM at PD7 (J7)
// to control a camera servo for nick compensation.
void TIMER2_Init(void)
{
	uint8_t sreg = SREG;

	// disable all interrupts before reconfiguration
	cli();

	PORTD &= ~(1<<PORTD7); 	// set PD7 to low

	DDRC  |= (1<<DDC6);     // set PC6 as output (Reset for HEF4017)
    HEF4017Reset_ON;
	// Timer/Counter 2 Control Register A

	// Timer Mode is FastPWM with timer reload at OCR2A (Bits: WGM22 = 1, WGM21 = 1, WGM20 = 1)
    // PD7: Normal port operation, OC2A disconnected, (Bits: COM2A1 = 0, COM2A0 = 0)
    // PD6: Normal port operation, OC2B disconnected, (Bits: COM2B1 = 0, COM2B0 = 0)
	TCCR2A &= ~((1<<COM2A1)|(1<<COM2A0)|(1<<COM2B1)|(1<<COM2B0));
    TCCR2A |= (1<<WGM21)|(1<<WGM20);

    // Timer/Counter 2 Control Register B

	// Set clock divider for timer 2 to SYSKLOCK/32 = 20MHz / 32 = 625 kHz
	// The timer increments from 0x00 to 0xFF with an update rate of 625 kHz or 1.6 us
	// hence the timer overflow interrupt frequency is 625 kHz / 256 = 2.44 kHz or 0.4096 ms

    // divider 32 (Bits: CS022 = 0, CS21 = 1, CS20 = 1)
	TCCR2B &= ~((1<<FOC2A)|(1<<FOC2B)|(1<<CS22));
    TCCR2B |= (1<<CS21)|(1<<CS20)|(1<<WGM22);

	// Initialize the Timer/Counter 2 Register
    TCNT2 = 0;

	// Initialize the Output Compare Register A used for PWM generation on port PD7.
	OCR2A = 255;
	TCCR2A |= (1<<COM2A1); // set or clear at compare match depends on value of COM2A0

	// Timer/Counter 2 Interrupt Mask Register
	// Enable timer output compare match A Interrupt only
	TIMSK2 &= ~((1<<OCIE2B)|(1<<TOIE2));
	TIMSK2 |= (1<<OCIE2A);

    SREG = sreg;
}

//----------------------------
void Timer_Init(void)
{
    tim_main = SetDelay(10);
    TCCR0B = CK8;
//    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|3;//fast PWM
    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<COM0B0)|3;//fast PWM
    OCR0B =  255;
    OCR0A = 180;
    TCNT0 = (unsigned char)-TIMER_RELOAD_VALUE;  // reload
    //OCR1  = 0x00;
    TIMSK0 |= _BV(TOIE0);
}


/*****************************************************/
/*              Control Servo Position               */
/*****************************************************/
void CalcNickServoValue(void)
{
 signed int max, min;

 if(EE_Parameter.ServoCompInvert & SERVO_RELATIVE) // relative moving of the servo value
  {
	max = ((unsigned int) EE_Parameter.ServoNickMax * MULTIPLYER * 15);
	min = ((unsigned int) EE_Parameter.ServoNickMin * MULTIPLYER * 20);
	NickServoValue -= ((signed char) (Parameter_ServoNickControl - 128) / 4) * 6;
	LIMIT_MIN_MAX(NickServoValue,min, max);
  }
  else NickServoValue = (int16_t)Parameter_ServoNickControl * (MULTIPLYER*16);  // direct poti control
}

void CalculateServo(void)
{
 signed char cosinus, sinus;
 signed long nick, roll;

	cosinus = sintab[EE_Parameter.CamOrientation + 6];
	sinus = sintab[EE_Parameter.CamOrientation];


  if(CalculateServoSignals == 1)
   {
	    if(EE_Parameter.GlobalConfig3 & CFG3_SERVO_NICK_COMP_OFF) nick = 0;
	    else nick = (cosinus * IntegralNick) / 128L - (sinus * IntegralRoll) / 128L;
        nick -= POI_KameraNick * 7;
		nick = ((long)Parameter_ServoNickComp * nick) / 512L;
		// offset (Range from 0 to 255 * 3 = 765)
		if(EE_Parameter.ServoCompInvert & SERVO_RELATIVE) ServoNickOffset = NickServoValue;
		else ServoNickOffset += (NickServoValue - ServoNickOffset) / EE_Parameter.ServoManualControlSpeed;

		if(EE_Parameter.ServoCompInvert & SERVO_NICK_INV) // inverting movement of servo
		{	
			nick = ServoNickOffset / 16 + nick;
		}
		else
		{	// inverting movement of servo
			nick = ServoNickOffset / 16 - nick;
		}
		if(EE_Parameter.ServoFilterNick) ServoNickValue = ((ServoNickValue * EE_Parameter.ServoFilterNick) + nick) / (EE_Parameter.ServoFilterNick + 1);
		else                     ServoNickValue = nick;
		// limit servo value to its parameter range definition
		if(ServoNickValue < ((int16_t)EE_Parameter.ServoNickMin * MULTIPLYER))
		{
			ServoNickValue = (int16_t)EE_Parameter.ServoNickMin * MULTIPLYER;
		}
		else
		if(ServoNickValue > ((int16_t)EE_Parameter.ServoNickMax * MULTIPLYER))
		{
			ServoNickValue = (int16_t)EE_Parameter.ServoNickMax * MULTIPLYER;
		}
//		if(PlatinenVersion < 20) CalculateServoSignals = 0; else 
		CalculateServoSignals++;
	}
	else
	{
	    if(EE_Parameter.GlobalConfig3 & CFG3_SERVO_NICK_COMP_OFF) roll = 0;
    	else roll = (cosinus * IntegralRoll) / 128L + (sinus * IntegralNick) / 128L;
    	roll = ((long)Parameter_ServoRollComp * roll) / 512L;
		ServoRollOffset += ((int16_t)Parameter_ServoRollControl * (MULTIPLYER*16) - ServoRollOffset) / EE_Parameter.ServoManualControlSpeed;
		if(EE_Parameter.ServoCompInvert & SERVO_ROLL_INV)
		{	// inverting movement of servo
			roll = ServoRollOffset / 16 + roll; 
		}
		else
		{	// inverting movement of servo
			roll = ServoRollOffset / 16 - roll; 
		}
		if(EE_Parameter.ServoFilterRoll) ServoRollValue = ((ServoRollValue * EE_Parameter.ServoFilterRoll) + roll) / (EE_Parameter.ServoFilterRoll + 1);
		else                     ServoRollValue = roll;
		// limit servo value to its parameter range definition
		if(ServoRollValue < ((int16_t)EE_Parameter.ServoRollMin * MULTIPLYER))
		{
			ServoRollValue = (int16_t)EE_Parameter.ServoRollMin * MULTIPLYER;
		}
		else
		if(ServoRollValue > ((int16_t)EE_Parameter.ServoRollMax * MULTIPLYER))
		{
			ServoRollValue = (int16_t)EE_Parameter.ServoRollMax * MULTIPLYER;
		}
		CalculateServoSignals = 0;
	}
}

ISR(TIMER2_COMPA_vect)
{
	// frame len 22.5 ms = 14063 * 1.6 us
	// stop pulse: 0.3 ms = 188 * 1.6 us
	// min servo pulse: 0.6 ms =  375 * 1.6 us
	// max servo pulse: 2.4 ms = 1500 * 1.6 us
	// resolution: 1500 - 375 = 1125 steps

	#define IRS_RUNTIME 127
	#define PPM_STOPPULSE 188
    #define PPM_FRAMELEN (1757 * EE_Parameter.ServoNickRefresh)
	#define MINSERVOPULSE 375
	#define MAXSERVOPULSE 1500
	#define SERVORANGE (MAXSERVOPULSE - MINSERVOPULSE)

	static uint8_t  PulseOutput = 0;
	static uint16_t ServoFrameTime = 0;
	static uint8_t  ServoIndex = 0;

/*
	if(PlatinenVersion < 20)
	{
		//---------------------------
		// Nick servo state machine
		//---------------------------
		if(!PulseOutput) // pulse output complete
		{
			if(TCCR2A & (1<<COM2A0)) // we had a low pulse
			{
				TCCR2A &= ~(1<<COM2A0);// make a high pulse
				RemainingPulse  = MINSERVOPULSE + SERVORANGE/2; // center position ~ 1.5ms
				RemainingPulse += ServoNickValue - (256 / 2) * MULTIPLYER; // shift ServoNickValue to center position
				// range servo pulse width
				if(RemainingPulse > MAXSERVOPULSE )			RemainingPulse = MAXSERVOPULSE; // upper servo pulse limit
				else if(RemainingPulse < MINSERVOPULSE )	RemainingPulse = MINSERVOPULSE; // lower servo pulse limit
				// accumulate time for correct update rate
				ServoFrameTime = RemainingPulse;
			}
			else // we had a high pulse
			{
				TCCR2A |= (1<<COM2A0); // make a low pulse
				RemainingPulse = PPM_FRAMELEN - ServoFrameTime;
				CalculateServoSignals = 1;
			}
			// set pulse output active
			PulseOutput = 1;
		}
	} // EOF Nick servo state machine
	else
*/
	{
		//-----------------------------------------------------
		// PPM state machine, onboard demultiplexed by HEF4017
		//-----------------------------------------------------
		if(!PulseOutput) // pulse output complete
		{
			if(TCCR2A & (1<<COM2A0)) // we had a low pulse
			{
				TCCR2A &= ~(1<<COM2A0);// make a high pulse
				if(ServoIndex == 0) // if we are at the sync gap
				{
					RemainingPulse = PPM_FRAMELEN - ServoFrameTime; // generate sync gap by filling time to full frame time
					ServoFrameTime = 0; // reset servo frame time
					HEF4017Reset_ON; // enable HEF4017 reset
				}
				else // servo channels 
				if(ServoIndex > EE_Parameter.ServoNickRefresh)  
				 {
				  RemainingPulse = 10; // end it here
				 } 
				else
				{
					RemainingPulse  = MINSERVOPULSE + SERVORANGE/2; // center position ~ 1.5ms
					if(ServoFailsafeActive && ServoIndex < 6 && EE_Parameter.ServoFS_Pos[ServoIndex-1]) RemainingPulse += ((int16_t)EE_Parameter.ServoFS_Pos[ServoIndex-1] * MULTIPLYER) - (256 / 2) * MULTIPLYER;
					else 
					switch(ServoIndex) // map servo channels
					{
					 case 1: // Nick Compensation Servo
							RemainingPulse += ServoNickValue - (256 / 2) * MULTIPLYER;
							break;
 					 case 2: // Roll Compensation Servo
							RemainingPulse += ServoRollValue - (256 / 2) * MULTIPLYER;
							break;
					 case 3:
					 		RemainingPulse += ((int16_t)Parameter_Servo3 * MULTIPLYER) - (256 / 2) * MULTIPLYER;
							break;
					 case 4:
					 		RemainingPulse += ((int16_t)Parameter_Servo4 * MULTIPLYER) - (256 / 2) * MULTIPLYER;
							break;
					 case 5:
					 		RemainingPulse += ((int16_t)Parameter_Servo5 * MULTIPLYER) - (256 / 2) * MULTIPLYER;
							break;
					default: // other servo channels
							RemainingPulse += 2 * PPM_in[ServoIndex]; // add channel value, factor of 2 because timer 1 increments 3.2µs
							break;
					}
					// range servo pulse width
					if(RemainingPulse > MAXSERVOPULSE)			RemainingPulse = MAXSERVOPULSE; // upper servo pulse limit
					else if(RemainingPulse < MINSERVOPULSE)	    RemainingPulse = MINSERVOPULSE; // lower servo pulse limit
					// substract stop pulse width
					RemainingPulse -= PPM_STOPPULSE;
					// accumulate time for correct sync gap
					ServoFrameTime += RemainingPulse;
				}
			}
			else // we had a high pulse
			{
				TCCR2A |= (1<<COM2A0); // make a low pulse
				// set pulsewidth to stop pulse width
				RemainingPulse = PPM_STOPPULSE;
				// accumulate time for correct sync gap
				ServoFrameTime += RemainingPulse;
				if((ServoActive/* && SenderOkay*/) || ServoActive == 2) HEF4017Reset_OFF; // disable HEF4017 reset
				else HEF4017Reset_ON;
				ServoIndex++; 
				if(ServoIndex > EE_Parameter.ServoNickRefresh+1)
				  {
				    CalculateServoSignals = 1;
					ServoIndex = 0; // reset to the sync gap
				  }
			}
			// set pulse output active
			PulseOutput = 1;
		}
	} // EOF PPM state machine

	// General pulse output generator
	if(RemainingPulse > (255 + IRS_RUNTIME))
	{
		OCR2A = 255;
		RemainingPulse -= 255;
	}
	else
	{
		if(RemainingPulse > 255) // this is the 2nd last part
		{
			if((RemainingPulse - 255) < IRS_RUNTIME)
			{
				OCR2A = 255 - IRS_RUNTIME;
				RemainingPulse -= 255 - IRS_RUNTIME;

			}
			else // last part > ISR_RUNTIME
			{
				OCR2A = 255;
				RemainingPulse -= 255;
			}
		}
		else // this is the last part
		{
			OCR2A = RemainingPulse;
			RemainingPulse = 0;
			PulseOutput = 0; // trigger to stop pulse
		}
	} // EOF general pulse output generator
}
