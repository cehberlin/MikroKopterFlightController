// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software Nutzungsbedingungen (english version: see below)
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
#include "jetimenu.h"
#include "libfc.h"
#include "printf_P.h"
#include "main.h"
#include "spi.h"
#include "capacity.h"
#include "hottmenu.h"

#define JETIBOX_KEY_RIGHT	0x1F
#define JETIBOX_KEY_UP	 	0x2F
#define JETIBOX_KEY_DOWN 	0x4F
#define JETIBOX_KEY_LEFT	0x8F
#define JETIBOX_KEY_NONE	0x0F
#define JETIBOX_KEY_UNDEF	0x00

#define JetiBox_printfxy(x,y,format, args...)  { LIBFC_JetiBox_SetPos(y * 16 + x); _printf_P(&LIBFC_JetiBox_Putchar, PSTR(format) , ## args);}
#define JetiBox_printf(format, args...)        {  _printf_P(&LIBFC_JetiBox_Putchar, PSTR(format) , ## args);}

unsigned char JumpToMenu = 0xff;

// -----------------------------------------------------------
// the menu functions
// -----------------------------------------------------------
void Menu_Status(uint8_t key)
{						//0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	JetiBox_printfxy(0,0,"%2i.%1iV",UBat/10, UBat%10);
	if(NaviDataOkay)
	{
		JetiBox_printfxy(6,0,"%3d%c %3dm%c",CompassCorrected, 0xDF, GPSInfo.HomeDistance/10,NC_GPS_ModeCharacter);
	}
	else
	{
		JetiBox_printfxy(6,0,"Status    ");
	}

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	if(NC_ErrorCode) 
	{
	 static unsigned int timer;
	 static char toggle = 1;
     
	 if(CheckDelay(timer)) { if(toggle) toggle = 0; else toggle = 1; timer = SetDelay(1500);};
     if(toggle)
	  {
       LIBFC_JetiBox_SetPos(0);
 	   _printf_P(&LIBFC_JetiBox_Putchar, NC_ERROR_TEXT[NC_ErrorCode] , 0); 
	  } 
	  else 
	  {
	   JetiBox_printfxy(6,0,"ERROR: %2d ",NC_ErrorCode);
//	   if(MotorenEin) JetiBeep = 'O'; 
	  } 
	}
	else 
	 if(NC_To_FC_Flags & NC_TO_FC_SIMULATION_ACTIVE)
	 {
	   JetiBox_printfxy(6,0,"SIMULATION");
	 }
	else 
	if(ShowSettingNameTime)
	{
	 LIBFC_JetiBox_Clear();
	 JetiBox_printfxy(0,0,"Set%d:%s",ActiveParamSet,EE_Parameter.Name); 
	 if(FC_StatusFlags3 & FC_STATUS3_BOAT) JetiBox_printfxy(0,1,"(Boat-Mode)"); 
	 return; // nichts weiter ausgeben
	}

#else
	if(NC_ErrorCode) { JetiBox_printfxy(6,0,"ERROR: %2d ",NC_ErrorCode); if(MotorenEin) JetiBeep = 'S';}; 
#endif
	JetiBox_printfxy(0,1,"%4i %2i:%02i",Capacity.UsedCapacity,FlugSekunden/60,FlugSekunden%60);
	if(Parameter_GlobalConfig & CFG_HOEHENREGELUNG)
	{
		JetiBox_printfxy(10,1,"%4im%c", (int16_t)(HoehenWert/100),VarioCharacter);
	}
	if(FC_StatusFlags3 & FC_STATUS3_REDUNDANCE_AKTIVE) JetiBox_printfxy(10,1,"R");

#endif
}


void Menu_Temperature(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
  JetiBox_printfxy(0,0,"%3i %3i %3i %3i", Motor[0].Temperature, Motor[1].Temperature, Motor[2].Temperature, Motor[3].Temperature);
  JetiBox_printfxy(0,1,"%3i %3i %3i %3i", Motor[4].Temperature, Motor[5].Temperature, Motor[6].Temperature, Motor[7].Temperature);
  if(RequiredMotors <= 4)
	{
	 JetiBox_printfxy(0,1,"Temperatures    ");
    }
	else
    if(RequiredMotors <= 6)
	{
	 JetiBox_printfxy(8,1,"\%cC     ",0xdf);
	} 

#endif
}

void Menu_Battery(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	JetiBox_printfxy(0,0,"%2i.%1iV %3i.%1iA", UBat/10, UBat%10, Capacity.ActualCurrent/10, Capacity.ActualCurrent%10);
	JetiBox_printfxy(0,1,"%4iW %6imAh",Capacity.ActualPower, Capacity.UsedCapacity);
#endif
}

void Magnet_Values(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	JetiBox_printfxy(0,0,"Magnet:%3i%% %3i%c",EarthMagneticField, KompassValue,0xDF);
	JetiBox_printfxy(0,1,"Incli.:%3i%c (%i) ",EarthMagneticInclination, 0xDF,EarthMagneticInclinationTheoretic);
#endif
}


void Menu_WPL_A1(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	JetiBox_printfxy(0,0,"Load Waypoints");
	JetiBox_printfxy(0,1,"(Fixed)      ");
#endif
}

void Menu_WPL_R1(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	JetiBox_printfxy(0,0,"Load Waypoints");
	JetiBox_printfxy(0,1,"(Relative)   ");
#endif
}

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
void Menu_POINT_LD(uint8_t key)
{                       //0123456789ABCDEF
	JetiBox_printfxy(0,0,"Load singl.Point");
//	JetiBox_printfxy(0,1,"(Fixed)      ");
}
#endif

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
void Menu_POINT_SV(uint8_t key)
{                       //0123456789ABCDEF
	JetiBox_printfxy(0,0,"Save singl.Point");
//	JetiBox_printfxy(0,1,"(Relative)   ");
}

void Menu_AccCal_Ask(uint8_t key)
{                       //0123456789ABCDEF
	JetiBox_printfxy(0,0,"ACC calibration");
//	JetiBox_printfxy(0,1,"(Relative)   ");
}

void Menu_AccCal(uint8_t key)
{ 
 static unsigned char changed = 0;
						//0123456789ABCDEF
	JetiBox_printfxy(0,0,"ACC calibration");

	if((FC_StatusFlags & FC_STATUS_MOTOR_RUN) && ((NC_GPS_ModeCharacter == ' ') || (NC_GPS_ModeCharacter == '/') || (NC_GPS_ModeCharacter == '-')))
	 {
	  if(!EE_Parameter.Driftkomp) EE_Parameter.Driftkomp = 6; // enables the Gyro-Drift compensation to make sure that a litlte calibration error won't effect the attitude					  
  	  JetiBox_printfxy(0,0,"ACC  N=%3i R=%3i",NeutralAccX,NeutralAccY);
	  if(ChannelNick || ChannelRoll) 
					JetiBox_printfxy(0,1,"Stick! (%i/%i)",ChannelNick,ChannelRoll)
	  else	                           //0123456789ABCDEF
	  if(changed) 	JetiBox_printfxy(0,1,"land to save    ")
	  else  		JetiBox_printfxy(0,1,"use keys now    ") 
	
 	  if(key== JETIBOX_KEY_UP   )  {NeutralAccX++;JetiBeep=130; changed = 1;}
  	  if(key== JETIBOX_KEY_DOWN )  {NeutralAccX--;JetiBeep=130; changed = 1;}
	  if(key== JETIBOX_KEY_RIGHT ) {NeutralAccY++;JetiBeep=130; changed = 1;}
	  if(key== JETIBOX_KEY_LEFT)   {NeutralAccY--;JetiBeep=130; changed = 1;}
    }
	 else
	 {
	  if(!(FC_StatusFlags & FC_STATUS_MOTOR_RUN))    // motors are off 
		 {
	     if(key == JETIBOX_KEY_LEFT) { JumpToMenu = 0; changed = 0; }// Exit

		 if(changed == 0) JetiBox_printfxy(0,1,"Fly with GPS off") 
		 else
		 if(changed == 1)
		   {
			  JetiBox_printfxy(0,1,"       save -->")
		    if(key== JETIBOX_KEY_RIGHT) 
			 {
			  StoreNeutralToEeprom(); 
			  JetiBeep = 130; 
			  changed = 2;
			 }
		  } 
		 else
		 if(changed == 2)
		   {
			  JetiBox_printfxy(0,1," values stored  ");
           }
	   }                        //0123456789ABCDEF
	   else JetiBox_printfxy(0,1,"switch GPS off  ") 
	 }  
//	JetiBox_printfxy(0,1,"(Relative)   ");
}
#endif

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
void Menu_POINT_SV2(uint8_t key)
{                       //0123456789ABCDEF
static unsigned char load_waypoint_tmp = 0, changed, hyterese = 1;
static int delay;
//  if(WPL_Name[0] == 0) JetiBox_printfxy(0,0,"Relative WPs ")
//  else JetiBox_printfxy(0,0,"Rel:%s",WPL_Name);
  JetiBox_printfxy(0,0,"Save Point:");
  
  if(NaviData_MaxWpListIndex == 0) JetiBox_printfxy(0,1,"no SD-Card")
  else
  if(GPSInfo.SatFix != SATFIX_3D) JetiBox_printfxy(0,1,"no GPS-Fix")
  else
  {
	if(load_waypoint_tmp)	JetiBox_printfxy(11,0,"%2d",load_waypoint_tmp)
	else JetiBox_printfxy(11,0,"--");

//	if(NaviData_WaypointNumber)	JetiBox_printfxy(8,1,"%2d/%d ",NaviData_WaypointIndex,NaviData_WaypointNumber)
//	else JetiBox_printfxy(8,1,"--/--")
	JetiBox_printfxy(0,1,"Dir:%3d Alt:%3dm",CompassCorrected,(int16_t)(HoehenWert/100))

    if(changed) 	JetiBox_printfxy(14,0,"->")
	else            JetiBox_printfxy(14,0,"  ");

	if(key == JETIBOX_KEY_UP && load_waypoint_tmp < NaviData_MaxWpListIndex) { load_waypoint_tmp++; changed = 1;}
	if(key == JETIBOX_KEY_DOWN && load_waypoint_tmp > 1) { load_waypoint_tmp--; changed = 1; }

	if(key == JETIBOX_KEY_RIGHT && load_waypoint_tmp)
	{
	 ToNC_Store_SingePoint = load_waypoint_tmp;
	 changed = 0;
	}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Bedienung per Taster am Sender
  if(PPM_in[EE_Parameter.MenuKeyChannel] > 50)  // 
   {
    hyterese = 2;
    if(CheckDelay(delay)) { load_waypoint_tmp = 0; hyterese = 1;}
   }
  else
  if(PPM_in[EE_Parameter.MenuKeyChannel] < -50)  
   {
	delay = SetDelay(2500);
	if(hyterese == 2 && (load_waypoint_tmp < NaviData_MaxWpListIndex))
	 {
	  load_waypoint_tmp++;
	  ToNC_Store_SingePoint = load_waypoint_tmp;
	  changed = 0;
//	  JetiBeep = 'A'; // "MikroKopter"
	 }
    hyterese = 0; 
   }
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  }
}
#endif

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
void Menu_POINT_LD2(uint8_t key)
{                       //0123456789ABCDEF
static unsigned char load_waypoint_tmp = 0, changed, hyterese = 1;
static int delay;  
//  if(WPL_Name[0] == 0) JetiBox_printfxy(0,0,"FIX Waypoints")
//  else JetiBox_printfxy(0,0,"FIX:%s",WPL_Name);
  JetiBox_printfxy(0,0,"Load Point")

  if(NaviData_MaxWpListIndex == 0) JetiBox_printfxy(0,1,"no SD-Card")
  else
  {
	if(load_waypoint_tmp)	JetiBox_printfxy(11,0,"%2d",load_waypoint_tmp)
	else JetiBox_printfxy(11,0,"--");

	if(NaviData_WaypointNumber)	JetiBox_printfxy(0,1,"Dist:%3d Alt:%3d ",NaviData_TargetDistance,(int16_t)(FromNC_AltitudeSetpoint/100))
	else JetiBox_printfxy(8,1,"                ");
	
    if(changed) 	JetiBox_printfxy(14,0,"->")
	else            JetiBox_printfxy(14,0,"  ");

	if(key == JETIBOX_KEY_UP && load_waypoint_tmp < NaviData_MaxWpListIndex) { load_waypoint_tmp++; changed = 1;}
	if(key == JETIBOX_KEY_DOWN && load_waypoint_tmp > 1) { load_waypoint_tmp--; changed = 1; }

	if(key == JETIBOX_KEY_RIGHT && load_waypoint_tmp)
	{
	 ToNC_Load_SingePoint = load_waypoint_tmp;
	 changed = 0;
	}
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Bedienung per Taster am Sender
  if(PPM_in[EE_Parameter.MenuKeyChannel] > 50)  // 
   {
    hyterese = 2;
    if(CheckDelay(delay)) { load_waypoint_tmp = 0; hyterese = 1;}
   }
  else
  if(PPM_in[EE_Parameter.MenuKeyChannel] < -50)  
   {
	delay = SetDelay(2500);
	if(hyterese == 2 && (load_waypoint_tmp < NaviData_MaxWpListIndex))
	 {
	  load_waypoint_tmp++;
	  ToNC_Load_SingePoint = load_waypoint_tmp;
	  changed = 0;
//	  JetiBeep = 'A'; // "MikroKopter"
	 }
    hyterese = 0; 
   }
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  }
}
#endif


void Menu_WPL_A2(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
static unsigned char load_waypoint_tmp = 1, changed;
  
  if(WPL_Name[0] == 0) JetiBox_printfxy(0,0,"FIX Waypoints")
  else JetiBox_printfxy(0,0,"FIX:%s",WPL_Name);

  if(NaviData_MaxWpListIndex == 0) JetiBox_printfxy(0,1,"no SD-Card")
  else
  {
	JetiBox_printfxy(0,1,"#%2d WP:",load_waypoint_tmp);
	if(NaviData_WaypointNumber)	JetiBox_printfxy(8,1,"%2d/%d ",NaviData_WaypointIndex,NaviData_WaypointNumber)
	else JetiBox_printfxy(8,1,"--/--")
	
    if(changed) 	JetiBox_printfxy(14,1,"->")
	else            JetiBox_printfxy(14,1,"  ");

	if(key == JETIBOX_KEY_UP && load_waypoint_tmp < NaviData_MaxWpListIndex) { load_waypoint_tmp++; changed = 1;}
	if(key == JETIBOX_KEY_DOWN && load_waypoint_tmp > 1) { load_waypoint_tmp--; changed = 1; }

	if(key == JETIBOX_KEY_RIGHT && load_waypoint_tmp)
	{
	 ToNC_Load_WP_List = load_waypoint_tmp;
	 changed = 0;
	}
  }
#endif
}

void Menu_WPL_R2(uint8_t key)
{                       //0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
static unsigned char load_waypoint_tmp = 1, changed;

  if(WPL_Name[0] == 0) JetiBox_printfxy(0,0,"Relative WPs ")
  else JetiBox_printfxy(0,0,"Rel:%s",WPL_Name);

  if(NaviData_MaxWpListIndex == 0) JetiBox_printfxy(0,1,"no SD-Card")
  else
  if(GPSInfo.SatFix != SATFIX_3D) JetiBox_printfxy(0,1,"no GPS-Fix")
  else
  {
	JetiBox_printfxy(0,1,"#%2d WPs:",load_waypoint_tmp);
	if(NaviData_WaypointNumber)	JetiBox_printfxy(8,1,"%2d/%d ",NaviData_WaypointIndex,NaviData_WaypointNumber)
	else JetiBox_printfxy(8,1,"--/--")

    if(changed) 	JetiBox_printfxy(14,1,"->")
	else            JetiBox_printfxy(14,1,"  ");

	if(key == JETIBOX_KEY_UP && load_waypoint_tmp < NaviData_MaxWpListIndex) { load_waypoint_tmp++; changed = 1;}
	if(key == JETIBOX_KEY_DOWN && load_waypoint_tmp > 1) { load_waypoint_tmp--; changed = 1; }

	if(key == JETIBOX_KEY_RIGHT && load_waypoint_tmp)
	{
	 ToNC_Load_WP_List = load_waypoint_tmp | 0x80;
	 changed = 0;
	}
  }
#endif
}



void Menu_PosInfo(uint8_t key)
{
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	if(NaviDataOkay)
	{
		JetiBox_printfxy(0,0,"%2um/s Sat:%d ",GPSInfo.Speed,GPSInfo.NumOfSats);
		switch (GPSInfo.SatFix)
		{
			case SATFIX_3D:
				JetiBox_printfxy(12,0,"  3D");
				break;
//			case SATFIX_2D:
//			case SATFIX_NONE:
			default:
				JetiBox_printfxy(12,0,"NoFx");
				break;
		}
		if(GPSInfo.Flags & FLAG_DIFFSOLN)
		{
			JetiBox_printfxy(12,0,"DGPS");
		}
		JetiBox_printfxy(0,1,"Home:%3dm %3d%c %c", GPSInfo.HomeDistance/10, GPSInfo.HomeBearing, 0xDF,NC_GPS_ModeCharacter);
	}
	else
	{                     //0123456789ABCDEF
		JetiBox_printfxy(2,0,"No NaviCtrl!");
	}
#endif
}


// -----------------------------------------------------------
// the menu topology
// -----------------------------------------------------------
typedef void (*pFctMenu) (uint8_t);  // the menu item handler function pointer

typedef struct{
  int8_t left;
  int8_t right;
  int8_t up;
  int8_t down;
  pFctMenu pHandler;
} MENU_ENTRY;


// the menu navigation structure
/*						|
3 - 0 - 1 - 2 - 3 - 0
*/

const MENU_ENTRY JetiBox_Menu[] PROGMEM=
{ // l  r  u  d  pHandler
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
#define ACC_CAL  13
	{8, 1, 0, 0, &Menu_Status }, 	// 0
	{0, 2, 1, 1, &Menu_Temperature },	// 1
	{1, 3, 2, 2, &Menu_Battery },	// 2
	{2, 4, 3, 3, &Menu_PosInfo },	// 3
	{3, 5, 4,10, &Menu_WPL_A1 },	// 4
	{4, 6, 5,11, &Menu_WPL_R1 },	// 5
	{5, 7, 6,12, &Menu_POINT_LD},	// 6
	{6, 8, 7,13, &Menu_POINT_SV},	// 7
	{7, 9, 8, 8, &Magnet_Values },	// 8
	{8, 0, 9,14, &Menu_AccCal_Ask},// 9

	{4,10,10,10, &Menu_WPL_A2 },	// 10
	{5,11,11,11, &Menu_WPL_R2 },	// 11
	{6,12,12,12, &Menu_POINT_LD2},	// 12
	{7,13,13,13, &Menu_POINT_SV2},	// 13

	{14,14,14,14, &Menu_AccCal},	// 14 
#else
	{6, 1, 0, 0, &Menu_Status }, 	// 0
	{0, 2, 1, 1, &Menu_Temperature },	// 1
	{1, 3, 2, 2, &Menu_Battery },	// 2
	{2, 4, 3, 3, &Menu_PosInfo },	// 3
	{3, 5, 7, 7, &Menu_WPL_A1 },	// 4
	{4, 6, 8, 8, &Menu_WPL_R1 },	// 5
	{5, 0, 6, 6, &Magnet_Values },	// 6

	{4, 7, 7, 7, &Menu_WPL_A2 },	// 7
	{5, 8, 8, 8, &Menu_WPL_R2 },	// 8
#endif
	
#endif
};

// -----------------------------------------------------------
// Update display buffer
// -----------------------------------------------------------
unsigned char JetiBox_Update(unsigned char key)
{
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	static uint8_t item = 0, last_item = 0; // the menu item
	static uint8_t updateDelay = 1 , last_key;
	// navigate within the menu by key action
	last_item = item;
	switch(key)
	{
		case JETIBOX_KEY_LEFT:
			//if (item == 0) return (1);									// switch back to jeti expander menu
			// else 
			 item = pgm_read_byte(&JetiBox_Menu[item].left);		//trigger to left menu item
			break;
		case JETIBOX_KEY_RIGHT:
			item = pgm_read_byte(&JetiBox_Menu[item].right);	//trigger to right menu item
			break;
		case JETIBOX_KEY_UP:
			item = pgm_read_byte(&JetiBox_Menu[item].up);		//trigger to up menu item
			break;
		case JETIBOX_KEY_DOWN:
			item = pgm_read_byte(&JetiBox_Menu[item].down);		//trigger to down menu item
			break;
		default:
			break;
	}
	if(JumpToMenu != 0xff) { item = JumpToMenu; JumpToMenu = 0xff;};
	// if the menu item has been changed, do not pass the key to the item handler
	// to avoid jumping over to items
	if(item != last_item) key = JETIBOX_KEY_UNDEF;

//	if((updateDelay++ & 0x01) || (key != last_key))
	if((updateDelay++ & 0x01) || (key != JETIBOX_KEY_NONE))
	{ 	
		last_key = key;
		LIBFC_JetiBox_Clear();
		//execute menu item handler
		((pFctMenu)(pgm_read_word(&(JetiBox_Menu[item].pHandler))))(key);
	}	
#endif
	return (0);
}

