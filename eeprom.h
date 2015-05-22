#ifndef _EEPROM_H
#define _EEPROM_H

#include <inttypes.h>
#include "twimaster.h"

#define EEPARAM_REVISION	 95 // is count up, if paramater stucture has changed (compatibility)
#define EEMIXER_REVISION	 1  // is count up, if mixer stucture has changed (compatibility)

#define EEPROM_ADR_PARAM_BEGIN		0
#define EE_DUMMY	 				0 // Byte
#define PID_EE_REVISION       		1 // byte
#define PID_ACTIVE_SET       		2 // byte
#define PID_PRESSURE_OFFSET  		3 // byte

#define PID_ACC_NICK         		4 // word
#define PID_ACC_ROLL         		6 // word
#define PID_ACC_TOP          		8 // word

#define PID_FLIGHT_MINUTES_TOTAL	10 // word
#define PID_FLIGHT_MINUTES		 	14 // word

#define PID_SPEAK_HOTT_CFG		 	16 // Byte
#define PID_HARDWARE_VERSION	 	17 // Byte

#define EEPROM_ADR_CHANNELS			80	// 80 - 93, 12 bytes + 1 byte crc
#define EEPROM_ADR_PARAMSET			100 // 100 - 725, 5 * 125 bytes
#define EEPROM_ADR_MIXERTABLE		1000 // 1000 - 1078, 78 bytes
#define EEPROM_ADR_BLCONFIG			1200 // 1200 - 1296, 12 * 8 bytes

#define MIX_GAS		0
#define MIX_NICK	1
#define MIX_ROLL	2
#define MIX_YAW		3

typedef struct
{
	uint8_t Revision;
    int8_t Name[12];
    int8_t Motor[16][4];
    uint8_t crc;
} __attribute__((packed)) MixerTable_t;

extern MixerTable_t Mixer;
extern uint8_t RequiredMotors;

//GlobalConfig3
#define CFG3_NO_SDCARD_NO_START  0x01
#define CFG3_DPH_MAX_RADIUS      0x02
#define CFG3_VARIO_FAILSAFE      0x04
#define CFG3_MOTOR_SWITCH_MODE   0x08
#define CFG3_NO_GPSFIX_NO_START  0x10
#define CFG3_USE_NC_FOR_OUT1     0x20
#define CFG3_SPEAK_ALL           0x40

//GlobalConfig
#define CFG_HOEHENREGELUNG       0x01
#define CFG_HOEHEN_SCHALTER      0x02
#define CFG_HEADING_HOLD         0x04
#define CFG_KOMPASS_AKTIV        0x08
#define CFG_KOMPASS_FIX          0x10
#define CFG_GPS_AKTIV            0x20
#define CFG_ACHSENKOPPLUNG_AKTIV 0x40
#define CFG_DREHRATEN_BEGRENZER  0x80

//BitConfig
#define CFG_LOOP_OBEN            0x01
#define CFG_LOOP_UNTEN           0x02
#define CFG_LOOP_LINKS           0x04
#define CFG_LOOP_RECHTS          0x08
#define CFG_MOTOR_BLINK1         0x10
#define CFG_MOTOR_OFF_LED1       0x20
#define CFG_MOTOR_OFF_LED2       0x40
#define CFG_MOTOR_BLINK2         0x80

// ExtraConfig
#define CFG2_HEIGHT_LIMIT       		0x01
#define CFG2_VARIO_BEEP       			0x02
#define CFG_SENSITIVE_RC     		    0x04
#define CFG_3_3V_REFERENCE   		    0x08
#define CFG_NO_RCOFF_BEEPING     		0x10
#define CFG_GPS_AID   		     		0x20
#define CFG_LEARNABLE_CAREFREE   		0x40
#define CFG_IGNORE_MAG_ERR_AT_STARTUP   0x80

// bit mask for ParamSet.Config0
#define CFG0_AIRPRESS_SENSOR		0x01
#define CFG0_HEIGHT_SWITCH			0x02
#define CFG0_HEADING_HOLD			0x04
#define CFG0_COMPASS_ACTIVE			0x08
#define CFG0_COMPASS_FIX			0x10
#define CFG0_GPS_ACTIVE				0x20
#define CFG0_AXIS_COUPLING_ACTIVE	0x40
#define CFG0_ROTARY_RATE_LIMITER	0x80

// bitcoding for EE_Parameter.ServoCompInvert
#define SERVO_NICK_INV 0x01
#define SERVO_ROLL_INV 0x02
#define SERVO_RELATIVE 0x04   //  direct poti control or relative moving of the servo value

// defines for the receiver selection
#define RECEIVER_PPM				0
#define RECEIVER_SPEKTRUM			1
#define RECEIVER_SPEKTRUM_HI_RES	2
#define RECEIVER_SPEKTRUM_LOW_RES 	3
#define RECEIVER_JETI				4
#define RECEIVER_ACT_DSL			5
#define RECEIVER_HOTT				6
#define RECEIVER_SBUS				7
#define RECEIVER_USER				8

#define RECEIVER_UNKNOWN			0xFF

// defines for lookup ParamSet.ChannelAssignment
#define K_NICK    0
#define K_ROLL    1
#define K_GAS     2
#define K_GIER    3
#define K_POTI1   4
#define K_POTI2   5
#define K_POTI3   6
#define K_POTI4   7
#define K_POTI5   8
#define K_POTI6   9
#define K_POTI7   10
#define K_POTI8   11


// values above 247 representing poti1 to poti8
// poti1 = 255
// poti2 = 254
// poti3 = 253
// poti4 = 252
// poti5 = 251
// poti6 = 250
// poti7 = 249
// poti8 = 248


typedef struct
{
	unsigned char Revision;
	unsigned char Kanalbelegung[12];       // GAS[0], GIER[1],NICK[2], ROLL[3], POTI1, POTI2, POTI3
	unsigned char GlobalConfig;           // 0x01=Höhenregler aktiv,0x02=Kompass aktiv, 0x04=GPS aktiv, 0x08=Heading Hold aktiv
	unsigned char Hoehe_MinGas;           // Wert : 0-100
	unsigned char Luftdruck_D;            // Wert : 0-250
	unsigned char HoeheChannel;           // Wert : 0-32
	unsigned char Hoehe_P;                // Wert : 0-32
	unsigned char Hoehe_Verstaerkung;     // Wert : 0-50
	unsigned char Hoehe_ACC_Wirkung;      // Wert : 0-250
	unsigned char Hoehe_HoverBand;        // Wert : 0-250
	unsigned char Hoehe_GPS_Z;            // Wert : 0-250
	unsigned char Hoehe_StickNeutralPoint;// Wert : 0-250
	unsigned char Stick_P;                // Wert : 1-6
	unsigned char Stick_D;                // Wert : 0-64
	unsigned char StickGier_P;            // Wert : 1-20
	unsigned char Gas_Min;                // Wert : 0-32
	unsigned char Gas_Max;                // Wert : 33-250
	unsigned char GyroAccFaktor;          // Wert : 1-64
	unsigned char KompassWirkung;         // Wert : 0-32
	unsigned char Gyro_P;                 // Wert : 10-250
	unsigned char Gyro_I;                 // Wert : 0-250
	unsigned char Gyro_D;                 // Wert : 0-250
	unsigned char Gyro_Gier_P;            // Wert : 10-250
	unsigned char Gyro_Gier_I;            // Wert : 0-250
	unsigned char Gyro_Stability;         // Wert : 0-16
	unsigned char UnterspannungsWarnung;  // Wert : 0-250
	unsigned char NotGas;                 // Wert : 0-250     //Gaswert bei Empängsverlust
	unsigned char NotGasZeit;             // Wert : 0-250     // Zeitbis auf NotGas geschaltet wird, wg. Rx-Problemen
	unsigned char Receiver;         	  // 0= Summensignal, 1= Spektrum, 2 =Jeti, 3=ACT DSL, 4=ACT S3D
	unsigned char I_Faktor;               // Wert : 0-250
	unsigned char UserParam1;             // Wert : 0-250
	unsigned char UserParam2;             // Wert : 0-250
	unsigned char UserParam3;             // Wert : 0-250
	unsigned char UserParam4;             // Wert : 0-250
	unsigned char ServoNickControl;       // Wert : 0-250     // Stellung des Servos
	unsigned char ServoNickComp;          // Wert : 0-250     // Einfluss Gyro/Servo
	unsigned char ServoNickMin;           // Wert : 0-250     // Anschlag
	unsigned char ServoNickMax;           // Wert : 0-250     // Anschlag
	//--- Seit V0.75
	unsigned char ServoRollControl;       // Wert : 0-250     // Stellung des Servos
	unsigned char ServoRollComp;          // Wert : 0-250
	unsigned char ServoRollMin;           // Wert : 0-250
	unsigned char ServoRollMax;           // Wert : 0-250
	//---
	unsigned char ServoNickRefresh;       // Speed of the Servo
    unsigned char ServoManualControlSpeed;//
    unsigned char CamOrientation;         //
	unsigned char Servo3;        		 // Value or mapping of the Servo Output
	unsigned char Servo4;       			 // Value or mapping of the Servo Output
	unsigned char Servo5;       			 // Value or mapping of the Servo Output
	unsigned char LoopGasLimit;           // Wert: 0-250  max. Gas während Looping
	unsigned char LoopThreshold;          // Wert: 0-250  Schwelle für Stickausschlag
	unsigned char LoopHysterese;          // Wert: 0-250  Hysterese für Stickausschlag
	unsigned char AchsKopplung1;          // Wert: 0-250  Faktor, mit dem Gier die Achsen Roll und Nick koppelt (NickRollMitkopplung)
	unsigned char AchsKopplung2;          // Wert: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
	unsigned char CouplingYawCorrection;  // Wert: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
	unsigned char WinkelUmschlagNick;     // Wert: 0-250  180°-Punkt
	unsigned char WinkelUmschlagRoll;     // Wert: 0-250  180°-Punkt
	unsigned char GyroAccAbgleich;        // 1/k  (Koppel_ACC_Wirkung)
	unsigned char Driftkomp;
	unsigned char DynamicStability;
	unsigned char UserParam5;             // Wert : 0-250
	unsigned char UserParam6;             // Wert : 0-250
	unsigned char UserParam7;             // Wert : 0-250
	unsigned char UserParam8;             // Wert : 0-250
	//---Output ---------------------------------------------
	unsigned char J16Bitmask;             // for the J16 Output
	unsigned char J16Timing;              // for the J16 Output
	unsigned char J17Bitmask;             // for the J17 Output
	unsigned char J17Timing;              // for the J17 Output
	// seit version V0.75c
	unsigned char WARN_J16_Bitmask;       // for the J16 Output
	unsigned char WARN_J17_Bitmask;       // for the J17 Output
	//---NaviCtrl---------------------------------------------
	unsigned char NaviOut1Parameter;      // for the J16 Output
	unsigned char NaviGpsModeChannel;     // Parameters for the Naviboard
	unsigned char NaviGpsGain;
	unsigned char NaviGpsP;
	unsigned char NaviGpsI;
	unsigned char NaviGpsD;
	unsigned char NaviGpsPLimit;
	unsigned char NaviGpsILimit;
	unsigned char NaviGpsDLimit;
	unsigned char NaviGpsA;
	unsigned char NaviGpsMinSat;
	unsigned char NaviStickThreshold;
	unsigned char NaviWindCorrection;
	unsigned char NaviAccCompensation;    // New since 0.86 -> was: SpeedCompensation
	unsigned char NaviOperatingRadius;
	unsigned char NaviAngleLimitation;
	unsigned char NaviPH_LoginTime;
	//---Ext.Ctrl---------------------------------------------
	unsigned char ExternalControl;         // for serial Control
	//---CareFree---------------------------------------------
	unsigned char OrientationAngle;        // Where is the front-direction?
	unsigned char CareFreeChannel;	   // switch for CareFree
    unsigned char MotorSafetySwitch;
    unsigned char MotorSmooth;
    unsigned char ComingHomeAltitude;
    unsigned char FailSafeTime;
    unsigned char MaxAltitude;
	unsigned char FailsafeChannel;         // if the value of this channel is > 100, the MK reports "RC-Lost"
	unsigned char ServoFilterNick;  
	unsigned char ServoFilterRoll;  
	unsigned char StartLandChannel;  
	unsigned char LandingSpeed;  
	//------------------------------------------------
	unsigned char BitConfig;          // (war Loop-Cfg) Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts / wird getrennt behandelt
	unsigned char ServoCompInvert;    // //  0x01 = Nick, 0x02 = Roll, 0x04 = relative moving // WICHTIG!!! am Ende lassen
	unsigned char ExtraConfig;        // bitcodiert
	unsigned char GlobalConfig3;      // bitcodiert
	char Name[12];
	unsigned char crc;				  // must be the last byte!
} paramset_t; // 127 bytes 

#define  PARAMSET_STRUCT_LEN  sizeof(paramset_t)

extern paramset_t EE_Parameter;

extern uint8_t RAM_Checksum(uint8_t* pBuffer, uint16_t len);

extern void ParamSet_Init(void);
extern void SetDefaultParameter(uint8_t set, uint8_t restore_channels);

extern uint8_t ParamSet_ReadFromEEProm(uint8_t setnumber);
extern uint8_t ParamSet_WriteToEEProm(uint8_t setnumber);

extern uint8_t GetActiveParamSet(void);
extern void SetActiveParamSet(uint8_t setnumber);

extern uint8_t MixerTable_ReadFromEEProm(void);
extern uint8_t MixerTable_WriteToEEProm(void);

extern uint8_t GetParamByte(uint16_t param_id);
extern void SetParamByte(uint16_t param_id, uint8_t value);
extern uint16_t GetParamWord(uint16_t param_id);
extern void SetParamWord(uint16_t param_id, uint16_t value);


#endif //_EEPROM_H
