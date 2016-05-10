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
#include "eeprom.h"
volatile int  Aktuell_Nick,Aktuell_Roll,Aktuell_Gier,Aktuell_ax, Aktuell_ay,Aktuell_az, UBat = 150;
volatile int  AdWertNickFilter = 0, AdWertRollFilter = 0, AdWertGierFilter = 0;
volatile int  HiResNick = 2500, HiResRoll = 2500;
volatile int  AdWertNick = 0, AdWertRoll = 0, AdWertGier = 0;
volatile int  AdWertAccRoll = 0,AdWertAccNick = 0,AdWertAccHoch = 0;
volatile long Luftdruck = 32000;
volatile long SummenHoehe = 0;
volatile long StartLuftdruck;
volatile unsigned int  MessLuftdruck = 1023;
unsigned char DruckOffsetSetting;
signed char ExpandBaro = 0;
volatile int VarioMeter = 0;
volatile unsigned int ZaehlMessungen = 0;
unsigned char AnalogOffsetNick = 115,AnalogOffsetRoll = 115,AnalogOffsetGier = 115;
volatile unsigned char AdReady = 1;
unsigned int BaroStep = 500;
long ExpandBaroStep = 0;
long HoehenWertF = 0;
long HoehenWert_Mess = 0;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
long HoehenWertF_Mess = 0; 
unsigned char CalAthmospheare = 16;
unsigned char AD_ACC_Y = 6;
unsigned char AD_ACC_X = 7;
#endif

//#######################################################################################
void ADC_Init(void)
//#######################################################################################
{
    ADMUX = 0;//Referenz ist extern
    ANALOG_ON;
}

#define DESIRED_H_ADC 800

void CalcExpandBaroStep(void)
{
  if(ACC_AltitudeControl) ExpandBaroStep = BaroStep * (long)ExpandBaro;
  else ExpandBaroStep = (16 * BaroStep) * (long)ExpandBaro - 4;
}

void SucheLuftruckOffset(void)
{
 unsigned int off;
 ExpandBaro = 0;
 CalcExpandBaroStep();
  off = GetParamByte(PID_PRESSURE_OFFSET);
  if(off < 240) off += 10;
  OCR0A = off;
  OCR0B = 255-off;
  Delay_ms_Mess(150);
  if(MessLuftdruck > DESIRED_H_ADC) off = 240;
  for(; off > 5; off--)
   {
    OCR0A = off;
    OCR0B = 255-off;
    Delay_ms_Mess(100);
    printf(".");
    if(MessLuftdruck > DESIRED_H_ADC) break;
   }
   DruckOffsetSetting = off;
   SetParamByte(PID_PRESSURE_OFFSET, off);
 if((EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG) && (DruckOffsetSetting < 10 || DruckOffsetSetting >= 230)) VersionInfo.HardwareError[0] |= FC_ERROR0_PRESSURE;

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + correction of the altitude error in higher altitudes
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 CalAthmospheare = 15;                                      // re-claibrated from 16 to 15 at 2.09 -> the baro-Altimeter was about 7% too high
 if(ACC_AltitudeControl)
  {
   if(PlatinenVersion < 23) { if(off < 140) CalAthmospheare += (160 - off) / 26; }
//   else { if(off < 170) CalAthmospheare += (188 - off) / 19; }
   else { if(off < 170) CalAthmospheare += (188 - off) / 15; } // rescaled at 2.09
  }
 Luftdruck = MessLuftdruck * CalAthmospheare;
#endif
 Delay_ms_Mess(300);
}

/*
void SucheGyroOffset(void)
{
 unsigned char i, ready = 0;
 int timeout;
 timeout = SetDelay(2000);
 for(i=140; i != 0; i--)
  {
   if(ready == 3 && i > 10) i = 9;
   ready = 0;
   if(AdWertNick < 1020) AnalogOffsetNick--; else if(AdWertNick > 1030) AnalogOffsetNick++; else ready++;
   if(AdWertRoll < 1020) AnalogOffsetRoll--; else if(AdWertRoll > 1030) AnalogOffsetRoll++; else ready++;
   if(AdWertGier < 1020) AnalogOffsetGier--; else if(AdWertGier > 1030) AnalogOffsetGier++; else ready++;
   I2C_Start(TWI_STATE_GYRO_OFFSET_TX);
   if(AnalogOffsetNick < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; AnalogOffsetNick = 10;}; if(AnalogOffsetNick > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; AnalogOffsetNick = 245;};
   if(AnalogOffsetRoll < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; AnalogOffsetRoll = 10;}; if(AnalogOffsetRoll > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; AnalogOffsetRoll = 245;};
   if(AnalogOffsetGier < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW;  AnalogOffsetGier = 10;}; if(AnalogOffsetGier > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW;  AnalogOffsetGier = 245;};
   while(twi_state) if(CheckDelay(timeout)) {printf("\n\r DAC or I2C ERROR! Check I2C, 3Vref, DAC and BL-Ctrl"); break;}
   AdReady = 0;
   ANALOG_ON;
   while(!AdReady);
   if(i<10) Delay_ms_Mess(10);
  }
   Delay_ms_Mess(70);
}
*/
/*
0  n
1  r
2     g
3     y
4     x
5  n
6  r
7     u
8     z
9     L
10 n
11 r
12    g
13    y
14    x
15 n
16 r
17    L
*/


//#######################################################################################
//
ISR(ADC_vect)
//#######################################################################################
{
    static unsigned char kanal=0,state = 0;
	static signed int subcount = 0;
    static signed int gier1, roll1, nick1, nick_filter, roll_filter;
	static signed int accy, accx;
	static long tmpLuftdruck = 0;
	static char messanzahl_Druck = 0;
    switch(state++)
        {
        case 0:
            nick1 = ADC;
            kanal = AD_ROLL;
            break;
        case 1:
            roll1 = ADC;
		    kanal = AD_GIER;
            break;
        case 2:
            gier1 = ADC;
            kanal = AD_ACC_Y;
            break;
        case 3:
            Aktuell_ay = NeutralAccY - ADC;
            accy = Aktuell_ay;
		    kanal = AD_ACC_X;
            break;
        case 4:
            Aktuell_ax = ADC - NeutralAccX;
            accx =  Aktuell_ax;
            kanal = AD_NICK;
            break;
        case 5:
            nick1 += ADC;
            kanal = AD_ROLL;
            break;
        case 6:
            roll1 += ADC;
            kanal = AD_UBAT;
            break;
        case 7:
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
            if(EE_Parameter.ExtraConfig & CFG_3_3V_REFERENCE) UBat = (3 * UBat + (11 * ADC) / 30) / 4; // there were some single FC2.1 with 3.3V reference
			else   
#endif
			 {
			  static unsigned int tmpVoltage = 0;
			  if(!tmpVoltage) tmpVoltage = (10 * ADC);
			  if(tmpVoltage <= (10 * ADC)) tmpVoltage += 2; else tmpVoltage -= 2;
              UBat = tmpVoltage / 31;
			 } 
		    kanal = AD_ACC_Z;
            break;
       case 8:
			 Aktuell_az = ADC;
			 AdWertAccHoch = Aktuell_az - NeutralAccZ - (int) NeutralAccZfine;
		     if(!ACC_AltitudeControl) // The Offset must be corrected, because of the ACC-Drift from vibrations
		     {
		      if(AdWertAccHoch > 1)
               {
                if(NeutralAccZ < 750)
                 {
                  subcount += 5;
                  if(modell_fliegt < 500) subcount += 10;
                  if(subcount > 100) { NeutralAccZ++; subcount -= 100;}
                 }
               }
               else if(AdWertAccHoch < -1)
               {
                if(NeutralAccZ > 550)
                 {
                  subcount -= 5;
                  if(modell_fliegt < 500) subcount -= 10;
                  if(subcount < -100) { NeutralAccZ--; subcount += 100;}
                 }
               }
             }
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			 else
			 if(CosAttitude > 8192 - 50) // horizontal leveled within 6°
		     {
		      if(AdWertAccHoch > 1)
               {
                  if(++subcount > 5000) 
				   { 
                    if(NeutralAccZfine < 6) NeutralAccZfine++; 
					subcount -= 5000;
				   }
               }
               else 
			   if(AdWertAccHoch < -1)
               {
                  if(--subcount < -5000) 
				   { 
                    if(NeutralAccZfine > -6) NeutralAccZfine--; 
					subcount += 5000;
				   }
               }
             }
#endif
            Mess_Integral_Hoch += AdWertAccHoch;      // Integrieren
            Mess_Integral_Hoch -= Mess_Integral_Hoch / 1024; // dämfen
 	        kanal = AD_DRUCK;
            break;
   // "case 9:" fehlt hier absichtlich
        case 10:
            nick1 += ADC;
            kanal = AD_ROLL;
            break;
        case 11:
            roll1 += ADC;
		    kanal = AD_GIER;
            break;
        case 12:
/*            if(PlatinenVersion == 10)  AdWertGier = (ADC + gier1 + 1) / 2;
            else
            if(PlatinenVersion >= 20)  AdWertGier = 2047 - (ADC + gier1);
			else 					   AdWertGier = (ADC + gier1);
*/
			AdWertGier = 2047 - (ADC + gier1);
            kanal = AD_ACC_Y;
            break;
        case 13:
            Aktuell_ay = NeutralAccY - ADC;
            AdWertAccRoll = (Aktuell_ay + accy);
            kanal = AD_ACC_X;
            break;
        case 14:
            Aktuell_ax = ADC - NeutralAccX;
            AdWertAccNick =  (Aktuell_ax + accx);
            kanal = AD_NICK;
            break;
        case 15:
            nick1 += ADC;
            //if(PlatinenVersion == 10) nick1 *= 2; else 
			nick1 *= 4;
            AdWertNick = nick1 / 8;
            nick_filter = (nick_filter + nick1) / 2;
            HiResNick = nick_filter - AdNeutralNick;
            AdWertNickFilter = (AdWertNickFilter + HiResNick) / 2;
            kanal = AD_ROLL;
            break;
        case 16:
            roll1 += ADC;
            //if(PlatinenVersion == 10) roll1 *= 2; else 
			roll1 *= 4;
            AdWertRoll = roll1 / 8;
            roll_filter = (roll_filter + roll1) / 2;
            HiResRoll = roll_filter - AdNeutralRoll;
            AdWertRollFilter = (AdWertRollFilter + HiResRoll) / 2;
 	        kanal = AD_DRUCK;
            break;
        case 17:
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			if(ACC_AltitudeControl)			
			{
			 HoehenWertF_Mess = (ACC_AltitudeFusion(0) + SA_FILTER/2)/SA_FILTER;	// cm
            }
			else HoehenWertF_Mess = HoehenWert;
#endif
            state = 0;
			AdReady = 1;
            ZaehlMessungen++;
            // "break" fehlt hier absichtlich
        case 9:
        	MessLuftdruck = ADC;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			if(ACC_AltitudeControl)			
			{
				tmpLuftdruck = MessLuftdruck - ExpandBaroStep;  // -523 counts per offset step
				if(BaroExpandActive)
				{
				  if(BaroExpandActive < 10) Luftdruck = tmpLuftdruck * CalAthmospheare;
				}
				else
				{
					Luftdruck -= Luftdruck / CalAthmospheare; // 16
					Luftdruck += tmpLuftdruck;
					HoehenWert_Mess = StartLuftdruck - Luftdruck;	// cm
				}			
			}
			else 
#endif
            {   // old version (until FC V2.1)
				tmpLuftdruck += MessLuftdruck;
				if(++messanzahl_Druck >= 16) // war bis 0.86 "18"
				{
			    signed int tmp;
				Luftdruck = (7 * Luftdruck + tmpLuftdruck - ExpandBaroStep) / 8;  // -523.19 counts per 10 counts offset step
				HoehenWert_Mess = StartLuftdruck - Luftdruck;
				SummenHoehe -= SummenHoehe/SM_FILTER;
				SummenHoehe += HoehenWert_Mess;
				tmp = (HoehenWert_Mess - SummenHoehe/SM_FILTER);
				if(tmp > 1024) tmp = 1024; 	else if(tmp < -1024) tmp = -1024; 
                if(abs(VarioMeter) > 700) VarioMeter = (15 * VarioMeter + 8 * tmp)/16;
				else VarioMeter = (31 * VarioMeter + 8 * tmp)/32;
                tmpLuftdruck /= 2;
                messanzahl_Druck = 16/2;
				}
			}
            kanal = AD_NICK;
            break;
        default:
            kanal = 0; state = 0; kanal = AD_NICK;
            break;
        }
    ADMUX = kanal;
    if(state != 0) ANALOG_ON;
}

