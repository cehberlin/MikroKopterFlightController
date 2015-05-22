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


#ifndef EEMEM
#define EEMEM __attribute__ ((section (".eeprom")))
#endif


#include <avr/eeprom.h>
#include <string.h>
#include "eeprom.h"
#include "uart.h"
#include "led.h"
#include "main.h"
#include "fc.h"
#include "twimaster.h"

paramset_t		EE_Parameter;
MixerTable_t	Mixer;
uint8_t RequiredMotors;


uint8_t RAM_Checksum(uint8_t* pBuffer, uint16_t len)
{
	uint8_t crc = 0xAA;
	uint16_t i;

	for(i=0; i<len; i++)
	{
		crc += pBuffer[i];
	}
	return crc;
}

uint8_t EEProm_Checksum(uint16_t EEAddr, uint16_t len)
{
	uint8_t crc = 0xAA;
	uint16_t off;

	for(off=0; off<len; off++)
	{
		crc += eeprom_read_byte((uint8_t*)(EEAddr + off));;
	}
	return crc;
}

void ParamSet_DefaultStickMapping(void)
{
	EE_Parameter.Kanalbelegung[K_GAS]   = 1;
	EE_Parameter.Kanalbelegung[K_ROLL]  = 2;
	EE_Parameter.Kanalbelegung[K_NICK]  = 3;
	EE_Parameter.Kanalbelegung[K_GIER]  = 4;
	EE_Parameter.Kanalbelegung[K_POTI1] = 5;
	EE_Parameter.Kanalbelegung[K_POTI2] = 6;
	EE_Parameter.Kanalbelegung[K_POTI3] = 7;
	EE_Parameter.Kanalbelegung[K_POTI4] = 8;
	EE_Parameter.Kanalbelegung[K_POTI5] = 9;
	EE_Parameter.Kanalbelegung[K_POTI6] = 10;
	EE_Parameter.Kanalbelegung[K_POTI7] = 11;
	EE_Parameter.Kanalbelegung[K_POTI8] = 12;
}


/***************************************************/
/*    Default Values for parameter set 1           */
/***************************************************/
void CommonDefaults(void)
{
	EE_Parameter.Revision = EEPARAM_REVISION;
	memset(EE_Parameter.Name,0,12); // delete name
	if(PlatinenVersion >= 20)
	{
		EE_Parameter.Gyro_D = 10;
		EE_Parameter.Driftkomp = 0;
		EE_Parameter.GyroAccFaktor = 27;
		EE_Parameter.WinkelUmschlagNick = 78;
		EE_Parameter.WinkelUmschlagRoll = 78;
	}
	else
	{
		EE_Parameter.Gyro_D = 3;
		EE_Parameter.Driftkomp = 32;
		EE_Parameter.GyroAccFaktor = 30;
		EE_Parameter.WinkelUmschlagNick = 85;
		EE_Parameter.WinkelUmschlagRoll = 85;
	}
	EE_Parameter.GyroAccAbgleich = 32;        // 1/k
	EE_Parameter.BitConfig = 0;              // Looping usw.
	EE_Parameter.GlobalConfig = CFG_ACHSENKOPPLUNG_AKTIV | CFG_KOMPASS_AKTIV | CFG_GPS_AKTIV | CFG_HOEHEN_SCHALTER;
	EE_Parameter.ExtraConfig = CFG_GPS_AID | CFG2_VARIO_BEEP | CFG_LEARNABLE_CAREFREE;
	EE_Parameter.Receiver = RECEIVER_HOTT;
	EE_Parameter.MotorSafetySwitch = 0; 
	EE_Parameter.ExternalControl = 0;

	EE_Parameter.Gas_Min = 8;             // Wert : 0-32
	EE_Parameter.Gas_Max = 230;           // Wert : 33-247
	EE_Parameter.KompassWirkung = 64;    // Wert : 0-247

	EE_Parameter.Hoehe_MinGas = 30;
	EE_Parameter.HoeheChannel = 5;         // Wert : 0-32
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	if(ACC_AltitudeControl)
	{
		EE_Parameter.Hoehe_P      = 20;          // Wert : 0-32
		EE_Parameter.Luftdruck_D  = 40;          // Wert : 0-247
		EE_Parameter.Hoehe_ACC_Wirkung = 30;     // Wert : 0-247
		EE_Parameter.Hoehe_HoverBand = 1;     	  // Wert : 0-247
		EE_Parameter.Hoehe_GPS_Z = 0;           // Wert : 0-247
		EE_Parameter.Hoehe_StickNeutralPoint = 127;// Wert : 0-247 (0 = Hover-Estimation)
		EE_Parameter.GlobalConfig3 = CFG3_SPEAK_ALL;//
		EE_Parameter.FailSafeTime = 30; 	          // 0 = off
	}
	else 
#endif
	{
		EE_Parameter.Hoehe_P      = 15;          // Wert : 0-32
		EE_Parameter.Luftdruck_D  = 30;          // Wert : 0-247
		EE_Parameter.Hoehe_ACC_Wirkung = 0;     // Wert : 0-247
		EE_Parameter.Hoehe_HoverBand = 8;     	  // Wert : 0-247
		EE_Parameter.Hoehe_GPS_Z = 20;           // Wert : 0-247
		EE_Parameter.Hoehe_StickNeutralPoint = 0;// Wert : 0-247 (0 = Hover-Estimation)
	    EE_Parameter.GlobalConfig3 = CFG3_SPEAK_ALL; 
		EE_Parameter.FailSafeTime = 0; 	          // 0 = off
	}
	
	EE_Parameter.Hoehe_Verstaerkung = 15;    // Wert : 0-50 (15 -> ca. +/- 5m/sek bei Stick-Voll-Ausschlag)
	EE_Parameter.StartLandChannel = 0;  
	EE_Parameter.LandingSpeed = 12;  

	EE_Parameter.UserParam1 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam2 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam3 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam4 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam5 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam6 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam7 = 0;             // zur freien Verwendung
	EE_Parameter.UserParam8 = 0;             // zur freien Verwendung

	EE_Parameter.ServoNickControl = 128;     // Wert : 0-247     // Stellung des Servos
	EE_Parameter.ServoNickComp = 50;         // Wert : 0-247     // Einfluss Gyro/Servo
	EE_Parameter.ServoCompInvert = 2;        // Wert : 0-247     // Richtung Einfluss Gyro/Servo
	EE_Parameter.ServoNickMin = 24;          // Wert : 0-247     // Anschlag
	EE_Parameter.ServoNickMax = 230;         // Wert : 0-247     // Anschlag
	EE_Parameter.ServoNickRefresh = 4;
	EE_Parameter.Servo3 = 125;
	EE_Parameter.Servo4 = 125;
	EE_Parameter.Servo5 = 125;
	EE_Parameter.ServoRollControl = 128;     // Wert : 0-247     // Stellung des Servos
	EE_Parameter.ServoRollComp = 85;         // Wert : 0-247     // Einfluss Gyro/Servo
	EE_Parameter.ServoRollMin = 70;          // Wert : 0-247     // Anschlag
	EE_Parameter.ServoRollMax = 220;         // Wert : 0-247     // Anschlag
	EE_Parameter.ServoManualControlSpeed = 60;
	EE_Parameter.CamOrientation = 0;         // Wert : 0-24 -> 0-360 -> 15° steps

	EE_Parameter.J16Bitmask = 95;
	EE_Parameter.J17Bitmask = 243;
	EE_Parameter.WARN_J16_Bitmask = 0xAA;
	EE_Parameter.WARN_J17_Bitmask = 0xAA;
	EE_Parameter.J16Timing = 40;
	EE_Parameter.J17Timing = 40;
    EE_Parameter.NaviOut1Parameter = 0;       // Photo release in meter
	EE_Parameter.LoopGasLimit = 50;
	EE_Parameter.LoopThreshold = 90;         // Wert: 0-247  Schwelle für Stickausschlag
	EE_Parameter.LoopHysterese = 50;

	EE_Parameter.NaviGpsModeChannel = 6; // Kanal 6
	EE_Parameter.NaviGpsGain = 100;
	EE_Parameter.NaviGpsP =  100;
	EE_Parameter.NaviGpsI =   90;
	EE_Parameter.NaviGpsD =  120;
	EE_Parameter.NaviGpsA =   40;
	EE_Parameter.NaviGpsPLimit = 75;
	EE_Parameter.NaviGpsILimit = 85;
	EE_Parameter.NaviGpsDLimit = 75;
	EE_Parameter.NaviGpsMinSat = 6;
	EE_Parameter.NaviStickThreshold = 8;
	EE_Parameter.NaviWindCorrection = 50;
	EE_Parameter.NaviAccCompensation = 42;
	EE_Parameter.NaviOperatingRadius = 245;
	EE_Parameter.NaviAngleLimitation = 140;
	EE_Parameter.NaviPH_LoginTime = 2;
	EE_Parameter.OrientationAngle = 0;
	EE_Parameter.CareFreeChannel = 0;
	EE_Parameter.UnterspannungsWarnung = 33; // Wert : 0-247 ( Automatische Zellenerkennung bei < 50)
	EE_Parameter.NotGas = 65;                // Wert : 0-247     // Gaswert bei Empangsverlust (ggf. in Prozent)
	EE_Parameter.NotGasZeit = 90;            // Wert : 0-247     // Zeit bis auf NotGas geschaltet wird, wg. Rx-Problemen
	EE_Parameter.MotorSmooth = 0;           
	EE_Parameter.ComingHomeAltitude = 0; 	  // 0 = don't change 
	EE_Parameter.MaxAltitude = 150;           // 0 = off
	EE_Parameter.AchsKopplung1 = 125;
	EE_Parameter.AchsKopplung2 = 52;
	EE_Parameter.FailsafeChannel = 0;
	EE_Parameter.ServoFilterNick = 0;
	EE_Parameter.ServoFilterRoll = 0;
}
/*
void ParamSet_DefaultSet1(void) // sport
{
	CommonDefaults();
	EE_Parameter.Stick_P = 14;            // Wert : 1-20
	EE_Parameter.Stick_D = 16;            // Wert : 0-20
	EE_Parameter.StickGier_P = 12;             // Wert : 1-20
	EE_Parameter.Gyro_P = 80;             // Wert : 0-247
	EE_Parameter.Gyro_I = 150;            // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 80;        // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 150;       // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	  // Wert : 1-8
	EE_Parameter.I_Faktor = 32;
	EE_Parameter.CouplingYawCorrection = 1;
	EE_Parameter.GyroAccAbgleich = 16;        // 1/k;
	EE_Parameter.DynamicStability = 100;
	memcpy(EE_Parameter.Name, "Sport\0", 12);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}
*/

/***************************************************/
/*    Default Values for parameter set 1           */
/***************************************************/
void ParamSet_DefaultSet1(void) // normal
{
	CommonDefaults();
	EE_Parameter.Stick_P = 10;               // Wert : 1-20
	EE_Parameter.Stick_D = 16;               // Wert : 0-20
	EE_Parameter.StickGier_P = 6;                 // Wert : 1-20
	EE_Parameter.Gyro_P = 90;                // Wert : 0-247
	EE_Parameter.Gyro_I = 120;               // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 90;           // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 120;          // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	  	  // Wert : 1-8
	EE_Parameter.I_Faktor = 32;
	EE_Parameter.CouplingYawCorrection = 60;
	EE_Parameter.DynamicStability = 75;
	memcpy(EE_Parameter.Name, "Fast",4);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}


/***************************************************/
/*    Default Values for parameter set 2           */
/***************************************************/
void ParamSet_DefaultSet2(void) // Agil
{
	CommonDefaults();
	EE_Parameter.Stick_P = 8;                // Wert : 1-20
	EE_Parameter.Stick_D = 16;               // Wert : 0-20
	EE_Parameter.StickGier_P  = 6;                // Wert : 1-20
	EE_Parameter.Gyro_P = 100;               // Wert : 0-247
	EE_Parameter.Gyro_I = 120;               // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 100;          // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 120;          // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	  	  // Wert : 1-8
	EE_Parameter.I_Faktor = 16;
	EE_Parameter.CouplingYawCorrection = 70;
	EE_Parameter.DynamicStability = 70;
	memcpy(EE_Parameter.Name, "Agile",5);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}

/***************************************************/
/*    Default Values for parameter set 3           */
/***************************************************/
void ParamSet_DefaultSet3(void) // Easy
{
	CommonDefaults();
	EE_Parameter.Stick_P = 6;                // Wert : 1-20
	EE_Parameter.Stick_D = 10;               // Wert : 0-20
	EE_Parameter.StickGier_P  = 4;           // Wert : 1-20
	EE_Parameter.Gyro_P = 100;               // Wert : 0-247
	EE_Parameter.Gyro_I = 120;               // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 100;          // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 120;          // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	      // Wert : 1-8
	EE_Parameter.I_Faktor = 16;
	EE_Parameter.CouplingYawCorrection = 70;
	EE_Parameter.DynamicStability = 70;
	memcpy(EE_Parameter.Name, "Easy", 4);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}


/***************************************************/
/*       Read Parameter from EEPROM as byte        */
/***************************************************/
uint8_t GetParamByte(uint16_t param_id)
{
	return eeprom_read_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + param_id));
}

/***************************************************/
/*       Write Parameter to EEPROM as byte         */
/***************************************************/
void SetParamByte(uint16_t param_id, uint8_t value)
{
	eeprom_write_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + param_id), value);
}

/***************************************************/
/*       Read Parameter from EEPROM as word        */
/***************************************************/
uint16_t GetParamWord(uint16_t param_id)
{
	return eeprom_read_word((uint16_t *)(EEPROM_ADR_PARAM_BEGIN + param_id));
}

/***************************************************/
/*       Write Parameter to EEPROM as word         */
/***************************************************/
void SetParamWord(uint16_t param_id, uint16_t value)
{
	eeprom_write_word((uint16_t*)(EEPROM_ADR_PARAM_BEGIN + param_id), value);
}

/***************************************************/
/*       Read Parameter Set from EEPROM            */
/***************************************************/
// number [1..5]
uint8_t ParamSet_ReadFromEEProm(uint8_t setnumber)
{
	uint8_t crc;
	uint16_t eeaddr;

	// range the setnumber
	if((1 > setnumber) || (setnumber > 5)) setnumber = 3;

	// calculate eeprom addr
	eeaddr = EEPROM_ADR_PARAMSET + PARAMSET_STRUCT_LEN * (setnumber - 1);

	// calculate checksum from eeprom
	crc = EEProm_Checksum(eeaddr, PARAMSET_STRUCT_LEN - 1);

	// check crc
	if(crc != eeprom_read_byte((uint8_t*)(eeaddr + PARAMSET_STRUCT_LEN - 1))) return 0;

	// check revision
	if(eeprom_read_byte((uint8_t*)(eeaddr)) != EEPARAM_REVISION) return 0;

	// read paramset from eeprom
	eeprom_read_block((void *) &EE_Parameter, (void*)(EEPROM_ADR_PARAMSET + PARAMSET_STRUCT_LEN * (setnumber - 1)), PARAMSET_STRUCT_LEN);
	LED_Init();
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	LIBFC_HoTT_Clear();
#endif
	return 1;
}

/***************************************************/
/*        Write Parameter Set to EEPROM            */
/***************************************************/
// number [1..5]
uint8_t ParamSet_WriteToEEProm(uint8_t setnumber)
{
	uint8_t crc;

	if(EE_Parameter.Revision == EEPARAM_REVISION) // write only the right revision to eeprom
	{
		if(setnumber > 5) setnumber = 5;
		if(setnumber < 1) return 0;
        LIBFC_CheckSettings(); 
		if(EE_Parameter.GlobalConfig3 & CFG3_VARIO_FAILSAFE)         // check the Setting: Not more than 100% emergency gas
			{
			 if(EE_Parameter.NotGas > 99) EE_Parameter.NotGas = 80; // i.e. 80% of Hovergas
			}
		// update checksum
		EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);

		// write paramset to eeprom
		eeprom_write_block((void *) &EE_Parameter, (void*)(EEPROM_ADR_PARAMSET + PARAMSET_STRUCT_LEN * (setnumber - 1)), PARAMSET_STRUCT_LEN);

		// backup channel settings to separate block in eeprom
		eeprom_write_block( (void*)(EE_Parameter.Kanalbelegung), (void*)(EEPROM_ADR_CHANNELS), sizeof(EE_Parameter.Kanalbelegung));

		// write crc of channel block to eeprom
		crc = RAM_Checksum((uint8_t*)(EE_Parameter.Kanalbelegung), sizeof(EE_Parameter.Kanalbelegung));
		eeprom_write_byte((uint8_t*)(EEPROM_ADR_CHANNELS + sizeof(EE_Parameter.Kanalbelegung)), crc);

		// update active settings number
		SetActiveParamSet(setnumber);
		LED_Init();
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
		LIBFC_HoTT_Clear();
#endif
		return 1;
	}
	// wrong revision
	return 0;
}

/***************************************************/
/*          Read MixerTable from EEPROM            */
/***************************************************/
uint8_t MixerTable_ReadFromEEProm(void)
{
	uint8_t crc;

	// calculate checksum in eeprom
	crc = EEProm_Checksum(EEPROM_ADR_MIXERTABLE, sizeof(Mixer) - 1);

	// check crc
	if( crc != eeprom_read_byte((uint8_t*)(EEPROM_ADR_MIXERTABLE + sizeof(Mixer) - 1)) ) return 0;

	// check revision
	if(eeprom_read_byte((uint8_t*)(EEPROM_ADR_MIXERTABLE)) != EEMIXER_REVISION) return 0;

	// read mixer table
	eeprom_read_block((void *) &Mixer, (void*)(EEPROM_ADR_MIXERTABLE), sizeof(Mixer));
	return 1;
}

/***************************************************/
/*          Write Mixer Table to EEPROM            */
/***************************************************/
uint8_t MixerTable_WriteToEEProm(void)
{
	if(Mixer.Revision == EEMIXER_REVISION)
	{
		// update crc
		Mixer.crc = RAM_Checksum((uint8_t*)(&Mixer), sizeof(Mixer) - 1);

		// write to eeprom
		eeprom_write_block((void *) &Mixer, (void*)(EEPROM_ADR_MIXERTABLE), sizeof(Mixer));
		return 1;
	}
	else return 0;
}

/***************************************************/
/*    Default Values for Mixer Table               */
/***************************************************/
void MixerTable_Default(void) // Quadro
{
	uint8_t i;

	Mixer.Revision = EEMIXER_REVISION;
	// clear mixer table
	for(i = 0; i < 16; i++)
	{
		Mixer.Motor[i][MIX_GAS]  = 0;
		Mixer.Motor[i][MIX_NICK] = 0;
		Mixer.Motor[i][MIX_ROLL] = 0;
		Mixer.Motor[i][MIX_YAW]  = 0;
	}
	// default = Quadro
	Mixer.Motor[0][MIX_GAS] = 64; Mixer.Motor[0][MIX_NICK] = +64; Mixer.Motor[0][MIX_ROLL] =   0; Mixer.Motor[0][MIX_YAW] = +64;
	Mixer.Motor[1][MIX_GAS] = 64; Mixer.Motor[1][MIX_NICK] = -64; Mixer.Motor[1][MIX_ROLL] =   0; Mixer.Motor[1][MIX_YAW] = +64;
	Mixer.Motor[2][MIX_GAS] = 64; Mixer.Motor[2][MIX_NICK] =   0; Mixer.Motor[2][MIX_ROLL] = -64; Mixer.Motor[2][MIX_YAW] = -64;
	Mixer.Motor[3][MIX_GAS] = 64; Mixer.Motor[3][MIX_NICK] =   0; Mixer.Motor[3][MIX_ROLL] = +64; Mixer.Motor[3][MIX_YAW] = -64;
	memcpy(Mixer.Name, "Quadro\0\0\0\0\0\0", 12);
	Mixer.crc = RAM_Checksum((uint8_t*)(&Mixer), sizeof(Mixer) - 1);
}

/***************************************************/
/*       Get active parameter set                  */
/***************************************************/
uint8_t GetActiveParamSet(void)
{
	uint8_t setnumber;
	setnumber = eeprom_read_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + PID_ACTIVE_SET));
	if(setnumber > 5)
	{
		setnumber = 3;
		eeprom_write_byte((void*)(EEPROM_ADR_PARAM_BEGIN+PID_ACTIVE_SET), setnumber);
	}
	ActiveParamSet = setnumber;
	return(setnumber);
}

/***************************************************/
/*       Set active parameter set                  */
/***************************************************/
void SetActiveParamSet(uint8_t setnumber)
{
	if(setnumber > 5) setnumber = 5;
	if(setnumber < 1) setnumber = 1;
	ActiveParamSet = setnumber;
	eeprom_write_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + PID_ACTIVE_SET), setnumber);
}

/***************************************************/
/*       Set default parameter set                 */
/***************************************************/
void SetDefaultParameter(uint8_t set, uint8_t restore_channels)
{

	if(set > 5) set = 5;
	else if(set < 1) set = 1;

	switch(set)
	{
		case 1:
			ParamSet_DefaultSet1(); // Fill ParamSet Structure to default parameter set 1 (Sport)
			break;
		case 2:
			ParamSet_DefaultSet2(); // Kamera
			break;
		case 3:
			ParamSet_DefaultSet3(); // Beginner
			break;
		default:
			ParamSet_DefaultSet3(); // Beginner
			break;
	}
	if(restore_channels)
	{
		uint8_t crc;
		// 1st check for a valid channel backup in eeprom
		crc = EEProm_Checksum(EEPROM_ADR_CHANNELS, sizeof(EE_Parameter.Kanalbelegung));
		if(crc == eeprom_read_byte((uint8_t*)(EEPROM_ADR_CHANNELS + sizeof(EE_Parameter.Kanalbelegung))) )
		{
			eeprom_read_block((void *)EE_Parameter.Kanalbelegung, (void*)(EEPROM_ADR_CHANNELS), sizeof(EE_Parameter.Kanalbelegung));
		}
		else ParamSet_DefaultStickMapping();
	}
	else ParamSet_DefaultStickMapping();
	ParamSet_WriteToEEProm(set);
}

/***************************************************/
/*       Initialize EEPROM Parameter Sets          */
/***************************************************/
void ParamSet_Init(void)
{
	uint8_t channel_backup  = 0, bad_params = 0, ee_default = 0,i;
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	if(PlatinenVersion != GetParamByte(PID_HARDWARE_VERSION)) 
	 {
	  if(PlatinenVersion == 22 && GetParamByte(PID_HARDWARE_VERSION) == 21 && !(PIND & 0x10)) SetParamByte(PID_EE_REVISION,0); // reset the Settings if the Version changed to V2.2
	  SetParamByte(PID_HARDWARE_VERSION,PlatinenVersion); // Remember the Version number
	  wdt_enable(WDTO_15MS); // Reset-Commando
	  printf("\n\r--> Hardware Version Byte Changed <--");
	  while(1);
	 } 
#endif
	if((EEPARAM_REVISION) != GetParamByte(PID_EE_REVISION))  
	{
		ee_default = 1; // software update or forced by mktool
	}
	// 1st check for a valid channel backup in eeprom
	i = EEProm_Checksum(EEPROM_ADR_CHANNELS, sizeof(EE_Parameter.Kanalbelegung));
	if(i == eeprom_read_byte((uint8_t*)(EEPROM_ADR_CHANNELS + sizeof(EE_Parameter.Kanalbelegung)))) channel_backup = 1;

	// parameter check

	// check all 5 parameter settings
	for (i = 1;i < 6; i++)
	{
		if(ee_default || !ParamSet_ReadFromEEProm(i)) // could not read paramset from eeprom
		{
			bad_params = 1;
			printf("\n\rGenerating default Parameter Set %d",i);
			switch(i)
			{
				case 1:
					ParamSet_DefaultSet1(); // Fill ParamSet Structure to default parameter set 1 (Sport)
					break;
				case 2:
					ParamSet_DefaultSet2(); // Normal
					break;
				default:
					ParamSet_DefaultSet3(); // Easy
					break;
			}
			if(channel_backup) // if we have an channel mapping backup in eeprom
			{	// restore it from eeprom
				eeprom_read_block((void *)EE_Parameter.Kanalbelegung, (void*)(EEPROM_ADR_CHANNELS), sizeof(EE_Parameter.Kanalbelegung));
			}
			else
			{	// use default mapping
				ParamSet_DefaultStickMapping();
			}
			ParamSet_WriteToEEProm(i);
		}
	}
	if(bad_params) // at least one of the parameter settings were invalid
	{
		// default-Setting is parameter set 3
		SetActiveParamSet(3);
	}
	// read active parameter set to ParamSet stucture
	i = GetActiveParamSet();
	ParamSet_ReadFromEEProm(i);
	printf("\n\rUsing Parameter Set %d", i);

	// load mixer table
	if(GetParamByte(PID_EE_REVISION) == 0xff || !MixerTable_ReadFromEEProm() )
	{
		printf("\n\rGenerating default Mixer Table");
		MixerTable_Default(); // Quadro
		MixerTable_WriteToEEProm();
	}
	if(ee_default)	SetParamByte(PID_EE_REVISION, (EEPARAM_REVISION));
	// determine motornumber
	RequiredMotors = 0;
	for(i = 0; i < 16; i++)
	{
		if(Mixer.Motor[i][MIX_GAS] > 0) RequiredMotors++;
	}

	printf("\n\rMixer-Config: '%s' (%u Motors)",Mixer.Name, RequiredMotors);
 PrintLine();// ("\n\r===================================");

}
