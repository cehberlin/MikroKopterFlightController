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

// -----------------------------------------------------------
// the menu functions
// -----------------------------------------------------------
void Menu_Status(uint8_t key)
{						//0123456789ABCDEF
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	JetiBox_printfxy(0,0,"%2i.%1iV",UBat/10, UBat%10);
	if(NaviDataOkay)
	{
		JetiBox_printfxy(6,0,"%3d%c %03dm%c",ErsatzKompassInGrad, 0xDF, GPSInfo.HomeDistance/10,NC_GPS_ModeCharacter);
	}
	else
	{
		JetiBox_printfxy(6,0,"Status");
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
	if(ShowSettingNameTime)
	{
	 LIBFC_JetiBox_Clear();
	 JetiBox_printfxy(0,1,"Set%d:%s  ",ActiveParamSet,EE_Parameter.Name); 
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
	{4, 1, 0, 0, &Menu_Status }, 	// 0
	{0, 2, 1, 1, &Menu_Temperature },	// 1
	{1, 3, 2, 2, &Menu_Battery },	// 2
	{2, 4, 3, 3, &Menu_PosInfo },	// 3
	{3, 0, 4, 4, &Magnet_Values }	// 4
#endif
};

// -----------------------------------------------------------
// Update display buffer
// -----------------------------------------------------------
unsigned char JetiBox_Update(unsigned char key)
{
#if !defined (RECEIVER_SPEKTRUM_DX7EXP) && !defined (RECEIVER_SPEKTRUM_DX8EXP)
	static uint8_t item = 0, last_item = 0; // the menu item
	static uint8_t updateDelay = 1;

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
	// if the menu item has been changed, do not pass the key to the item handler
	// to avoid jumping over to items
	if(item != last_item) key = JETIBOX_KEY_UNDEF;

	if (updateDelay++ & 0x01) 
	{ 	

		LIBFC_JetiBox_Clear();
		//execute menu item handler
		((pFctMenu)(pgm_read_word(&(JetiBox_Menu[item].pHandler))))(key);
	}	
#endif
	return (0);
}

