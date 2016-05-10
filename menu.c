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

char DisplayBuff[80];
unsigned char DispPtr = 0;

unsigned char MaxMenue = 19;
unsigned char MenuePunkt = 0;
unsigned char RemoteKeys = 0;

#define KEY1    0x01
#define KEY2    0x02
#define KEY3    0x04
#define KEY4    0x08
#define KEY5    0x10

void LcdClear(void)
{
 unsigned char i;
 for(i=0;i<80;i++) DisplayBuff[i] = ' ';
}

void Menu_Putchar(char c)
{
 if(DispPtr < 80) DisplayBuff[DispPtr++] = c;
}

void Menu(void)
 {
  unsigned char i;
  static char DisableMenu = 0, AccMenu = 0, changed = 0;

if(!DisableMenu)
 {
  if(RemoteKeys & KEY1) { if(MenuePunkt) MenuePunkt--; else MenuePunkt = MaxMenue;}
  if(RemoteKeys & KEY2) { if(MenuePunkt == MaxMenue) MenuePunkt = 0; else MenuePunkt++;}
  if((RemoteKeys & KEY1) && (RemoteKeys & KEY2)) MenuePunkt = 0;
 }
  LcdClear();
  if(MenuePunkt < 10) {LCD_printfxy(17,0,"[%i]",MenuePunkt);}
  else {LCD_printfxy(16,0,"[%i]",MenuePunkt);};

  switch(MenuePunkt)
   {
    case 0:
           LCD_printfxy(0,0,"+ MikroKopter +");
           LCD_printfxy(0,1,"HW:V%d.%d SW:%d.%02d%c ",PlatinenVersion/10,PlatinenVersion%10, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH +'a');
           LCD_printfxy(0,2,"Set:%d %s", ActiveParamSet,Mixer.Name);
           if(FC_StatusFlags3 & FC_STATUS3_BOAT) LCD_printfxy(16,2,"BOAT");

            if(VersionInfo.HardwareError[1] & FC_ERROR1_MIXER) LCD_printfxy(0,3,"Mixer Error!")
		    else
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
			if(NC_ErrorCode) 
			{
				LCD_printfxy(0,3,"ERR%2d:",NC_ErrorCode);
				_printf_P(&Menu_Putchar, NC_ERROR_TEXT[NC_ErrorCode] , 0); 
			}
		   else
#endif
		   if(VersionInfo.HardwareError[0]) LCD_printfxy(0,3,"Hardware Error 1:%d !!",VersionInfo.HardwareError[0])
		   else
           if(MissingMotor) LCD_printfxy(0,3,"Missing BL-Ctrl:%d !!",MissingMotor)
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
#else
		   else
			if(NC_ErrorCode) 
			{
				LCD_printfxy(0,3,"! NC-ERR: %2d ! ",NC_ErrorCode);
			}
#endif
//		   if(VersionInfo.HardwareError[1]) LCD_printfxy(0,3,"Error 2:%d !!",VersionInfo.HardwareError[1])
		   else
           if(I2CTimeout < 6) LCD_printfxy(0,3,"I2C ERROR!!!")
		   else
  	       if(Capacity.MinOfMaxPWM < 30) LCD_printfxy(0,3,"BL-Selftest Err:%2d",Capacity.MinOfMaxPWM)

           break;
    case 1:
           LCD_printfxy(0,0,"Height:  %5i",(int)(HoehenWert/5));
           LCD_printfxy(0,1,"Setpoint:%5i",(int)(SollHoehe/5));
           LCD_printfxy(0,2,"Pressure:%5i",MessLuftdruck);
           LCD_printfxy(0,3,"Offset:  %5i",OCR0A);
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
		   if(ACC_AltitudeControl)	
		    {
			 LCD_printfxy(17,2,"(A)");
			 LCD_printfxy(17,3,"%i",CalAthmospheare);
			} 
#endif
           break;
    case 2:
           LCD_printfxy(0,0,"act. bearing");
           LCD_printfxy(0,1,"Nick:      %5i",IntegralNick/1024);
           LCD_printfxy(0,2,"Roll:      %5i",IntegralRoll/1024);
           LCD_printfxy(0,3,"Compass:   %5i",CompassCorrected);
           break;
    case 3:
           for(i=1;i<9;i+=2) LCD_printfxy(0,i/2,"K%i:%4i  K%i:%4i ",i,PPM_in[i],i+1,PPM_in[i+1]);
           break;
    case 4:
           LCD_printfxy(0,0,"Ni:%4i  Ro:%4i ",ChannelNick,ChannelRoll);
           LCD_printfxy(0,1,"Gs:%4i  Gi:%4i ",ChannelGas+127,ChannelYaw);
           LCD_printfxy(0,2,"P1:%4i  P2:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_POTI1]]+127,PPM_in[EE_Parameter.Kanalbelegung[K_POTI2]]+127);
           LCD_printfxy(0,3,"P3:%4i  P4:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_POTI3]]+127,PPM_in[EE_Parameter.Kanalbelegung[K_POTI4]]+127);
           break;
    case 5:
           LCD_printfxy(0,0,"Gyro - Sensor");
           LCD_printfxy(0,1,"Nick %4i (%4i)",AdWertNick - AdNeutralNick/8, AdNeutralNick);
           LCD_printfxy(0,2,"Roll %4i (%4i)",AdWertRoll - AdNeutralRoll/8, AdNeutralRoll);
           LCD_printfxy(0,3,"Yaw  %4i (%4i)",AdNeutralGier - AdWertGier, AdNeutralGier);
		   AccMenu = 0;
           break;
    case 6:
		switch(AccMenu)
		 {
			case 0:
					LCD_printfxy(0,0,"ACC - Sensor");
					LCD_printfxy(0,1,"Nick %4i (%3i)",AdWertAccNick,NeutralAccX);
					LCD_printfxy(0,2,"Roll %4i (%3i)",AdWertAccRoll,NeutralAccY);
					LCD_printfxy(0,3,"Z    %4i (%3i)(cal)",AdWertAccHoch,(int)NeutralAccZ);
				    if(RemoteKeys & KEY4) AccMenu++;
					changed = 0;
					break;
			case 1:		
					LCD_printfxy(0,0,"Calibration:");
  					LCD_printfxy(0,1,"ACC:   N:%3i R:%3i ",NeutralAccY,NeutralAccX);
					if(ChannelNick || ChannelRoll)	LCD_printfxy(0,2,"Stick: (%i/%i) !! ",ChannelNick,ChannelRoll);
					if((FC_StatusFlags & FC_STATUS_MOTOR_RUN) && ((NC_GPS_ModeCharacter == ' ') || (NC_GPS_ModeCharacter == '/') || (NC_GPS_ModeCharacter == '-')))
					 {
					  DisableMenu = 1;
					  if(!EE_Parameter.Driftkomp) EE_Parameter.Driftkomp = 6; // enables the Gyro-Drift compensation to make sure that a litlte calibration error won't effect the attitude					  
					  if(RemoteKeys & KEY1) { NeutralAccY--; changed = 1;}
					  if(RemoteKeys & KEY2) { NeutralAccY++; changed = 1;}
					  if(RemoteKeys & KEY3) { NeutralAccX--; changed = 1;}
					  if(RemoteKeys & KEY4) { NeutralAccX++; changed = 1;}
					  LCD_printfxy(13,0,"on ");
					  if(changed) 	LCD_printfxy(0,3,"land to save    ")
					  else LCD_printfxy(0,3,"use keys to trim ACC")
					 }
					 else
					 {
					  LCD_printfxy(13,0,"off");
				      DisableMenu = 0;
					  if(!(FC_StatusFlags & FC_STATUS_MOTOR_RUN) && changed) 
					   {
					  	LCD_printfxy(0,3,"Save?     (yes)(no)");
					    if(RemoteKeys & KEY3) { StoreNeutralToEeprom(); AccMenu = 0;}
					    if(RemoteKeys & KEY4) { changed = 0;}
					   }
					  else LCD_printfxy(0,3,"Fly with GPS off");					   
					 }
					break;
			default: AccMenu = 0; break;
		}
           break;
    case 7:
			LCD_printfxy(0,0,"Voltage:   %3i.%1iV",UBat/10, UBat%10);
			LCD_printfxy(0,1,"Current:   %3i.%1iA",Capacity.ActualCurrent/10, Capacity.ActualCurrent%10);
			LCD_printfxy(0,2,"Power:     %4iW",Capacity.ActualPower);
			LCD_printfxy(0,3,"Discharge: %5imAh", Capacity.UsedCapacity);
			AccMenu = 0;
    	   break;
    case 8:
           LCD_printfxy(0,0,"Receiver");
           LCD_printfxy(0,2,"RC-Quality: %4i", SenderOkay);
           LCD_printfxy(0,3,"RC-Channels:%4i", Channels-1);
           break;
    case 9:
           LCD_printfxy(0,0,"Undervoltages  " );
           LCD_printfxy(0,1,"Warn: %2d.%dV",BattLowVoltageWarning/10,BattLowVoltageWarning%10);
           LCD_printfxy(0,2,"Home: %2d.%dV",BattComingHomeVoltage/10,BattComingHomeVoltage%10);
           LCD_printfxy(0,3,"Land: %2d.%dV",BattAutoLandingVoltage/10,BattAutoLandingVoltage%10);
           break;
    case 10:
           for(i=0;i<4;i++) LCD_printfxy(0,i,"Poti%i:  %3i",i+1,Poti[i]);
           break;
    case 11:
           for(i=0;i<4;i++) LCD_printfxy(0,i,"Poti%i:  %3i",i+5,Poti[i+4]);
           break;
    case 12:
           LCD_printfxy(0,0,"Flight-Time  " );
           LCD_printfxy(0,1,"Total:%5umin",FlugMinutenGesamt);
           LCD_printfxy(0,2,"Act:  %5umin",FlugMinuten);
           LCD_printfxy(13,3,"(reset)");
		   if(RemoteKeys & KEY4)
             {
               FlugMinuten = 0;
               SetParamWord(PID_FLIGHT_MINUTES, FlugMinuten);
			 }
           break;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
    case 13:
           LCD_printfxy(0,0,"Compass");
           LCD_printfxy(0,1,"Magnet:   %5i",KompassValue);
           LCD_printfxy(0,2,"Gyro:     %5i",ErsatzKompassInGrad);
           LCD_printfxy(0,3,"True:     %5i",CompassCorrected);
           break;
    case 14:
           LCD_printfxy(0,0,"Servo  " );
           LCD_printfxy(0,1,"Setpoint  %3i",Parameter_ServoNickControl);
           LCD_printfxy(0,2,"Position: %3i",ServoNickValue/4);
           LCD_printfxy(0,3,"Range:%3i-%3i",EE_Parameter.ServoNickMin,EE_Parameter.ServoNickMax);
           break;
    case 15:
		   if(RequiredMotors < 13)
		    {
			 LCD_printfxy(0,0,"BL-Ctrl Errors " );
			 for(i=0;i<3;i++)                                                               
				LCD_printfxy(0,i+1,"%3d %3d %3d %3d ",Motor[i*4].State & MOTOR_STATE_ERROR_MASK,Motor[i*4+1].State & MOTOR_STATE_ERROR_MASK,Motor[i*4+2].State & MOTOR_STATE_ERROR_MASK,Motor[i*4+3].State & MOTOR_STATE_ERROR_MASK);
			}
			else
		    {
			 for(i=0;i<4;i++)                                                               
				LCD_printfxy(0,i,"%3d %3d %3d %3d ",Motor[i*4].State & MOTOR_STATE_ERROR_MASK,Motor[i*4+1].State & MOTOR_STATE_ERROR_MASK,Motor[i*4+2].State & MOTOR_STATE_ERROR_MASK,Motor[i*4+3].State & MOTOR_STATE_ERROR_MASK);
             LCD_printfxy(16,1,"BL-");
             LCD_printfxy(16,2,"Err.");
			}
           break;
    case 16:
		   if(RequiredMotors < 13)
		   {
            LCD_printfxy(0,0,"BL- Temperature" );
		     for(i=0;i<3;i++) 
		      {
		       LCD_printfxy(0,i+1,"%3d %3d %3d %3d ",Motor[i*4].Temperature,Motor[i*4+1].Temperature,Motor[i*4+2].Temperature,Motor[i*4+3].Temperature);
			  }			
			}
			else
		   {
		     for(i=0;i<4;i++) 
		      {
		       LCD_printfxy(0,i,"%3d %3d %3d %3d ",Motor[i*4].Temperature,Motor[i*4+1].Temperature,Motor[i*4+2].Temperature,Motor[i*4+3].Temperature);
			  }			
            LCD_printfxy(16,1,"BL-");
            LCD_printfxy(16,2,"Temp");
			}
            LCD_printfxy(18,3,"C");
           break;
    case 17:
		   if(RequiredMotors < 13)
		   {
			LCD_printfxy(0,0,"BL Current" );
			LCD_printfxy(11,3,"(in 0.1A)" );
			for(i=0;i<3;i++) 
				{
					LCD_printfxy(0,i+1,"%3d %3d %3d %3d ",BL3_Current(i*4),BL3_Current(i*4+1),BL3_Current(i*4+2),BL3_Current(i*4+3));
					if(4 + i * 4 >= RequiredMotors) break;
				}			
			}
			else
			{
			for(i=0;i<4;i++) 
				{
					LCD_printfxy(0,i,"%3d %3d %3d %3d ",BL3_Current(i*4),BL3_Current(i*4+1),BL3_Current(i*4+2),BL3_Current(i*4+3));
				}			
            LCD_printfxy(16,2,"BL-");
            LCD_printfxy(16,3,"I");
			}
           break;
    case 18:
            LCD_printfxy(0,0,"BL-Ctrl found " );
			DispPtr = 20;
			for(i=0; i<MAX_MOTORS; i++)
			{
			 if(i == 6 || i == 12) LCD_printf("  ");
			 if(Motor[i].State & MOTOR_STATE_PRESENT_MASK) 
			  {
				LCD_printf(" %2i",i+1);
			  } 
			 else LCD_printf("  -");
			}
           break;
    case 19:
		   if(RequiredMotors < 13)
		   {
			LCD_printfxy(0,0,"BL Versions" );
			for(i=0;i<3;i++) 
				{
					LCD_printfxy(0,i+1,"%1d.%02d %1d.%02d %1d.%02d %1d.%02d",Motor[i*4].VersionMajor,Motor[i*4].VersionMinor,Motor[i*4+1].VersionMajor,Motor[i*4+1].VersionMinor,Motor[i*4+2].VersionMajor,Motor[i*4+2].VersionMinor,Motor[i*4+3].VersionMajor,Motor[i*4+3].VersionMinor);
					if(4 + i * 4 >= RequiredMotors) break;
				}			
			}
			else
			{
			for(i=0;i<4;i++) 
				LCD_printfxy(0,i," %1d.%02d %1d.%02d %1d.%02d %1d.%02d",Motor[i*4].VersionMajor,Motor[i*4].VersionMinor,Motor[i*4+1].VersionMajor,Motor[i*4+1].VersionMinor,Motor[i*4+2].VersionMajor,Motor[i*4+2].VersionMinor,Motor[i*4+3].VersionMajor,Motor[i*4+3].VersionMinor);
			}
           break;
/*    case 20:
           LCD_printfxy(0,0,"BL MaxPWM " );
		   for(i=0;i<3;i++) 
		    {
		     LCD_printfxy(0,i+1,"%3d %3d %3d %3d ",Motor[i*4].MaxPWM,Motor[i*4+1].MaxPWM,Motor[i*4+2].MaxPWM,Motor[i*4+3].MaxPWM);
			 if(4 + i * 4 >= RequiredMotors) break;
			}			
           break;
*/
#endif
    default:
           if(MenuePunkt == MaxMenue) MaxMenue--;
    	   MenuePunkt = 0;
           break;
    }
    RemoteKeys = 0;
}
