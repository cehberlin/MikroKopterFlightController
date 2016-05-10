/*#######################################################################################
Flight Control
#######################################################################################*/

#ifndef _FC_H
#define _FC_H
//#define GIER_GRAD_FAKTOR 1291L // Abhängigkeit zwischen GyroIntegral und Winkel
//#define GIER_GRAD_FAKTOR 1160L
extern long GIER_GRAD_FAKTOR; // Abhängigkeit zwischen GyroIntegral und Winkel
#define STICK_GAIN 4
#define ACC_AMPLIFY    6
#define HEIGHT_CONTROL_STICKTHRESHOLD 15
#define SERVO_FS_TIME 10   // in Seconds

// FC_StatusFlags
#define FC_STATUS_MOTOR_RUN  				0x01
#define FC_STATUS_FLY        				0x02
#define FC_STATUS_CALIBRATE  				0x04
#define FC_STATUS_START      				0x08
#define FC_STATUS_EMERGENCY_LANDING      	0x10
#define FC_STATUS_LOWBAT		      		0x20
#define FC_STATUS_VARIO_TRIM_UP      		0x40
#define FC_STATUS_VARIO_TRIM_DOWN      		0x80

// FC_StatusFlags2 
#define FC_STATUS2_CAREFREE                 0x01
#define FC_STATUS2_ALTITUDE_CONTROL         0x02
#define FC_STATUS2_RC_FAILSAVE_ACTIVE       0x04
#define FC_STATUS2_OUT1_ACTIVE       		0x08
#define FC_STATUS2_OUT2_ACTIVE       		0x10
#define FC_STATUS2_WAIT_FOR_TAKEOFF       	0x20   // Motor Running, but still on the ground
#define FC_STATUS2_AUTO_STARTING       		0x40
#define FC_STATUS2_AUTO_LANDING       		0x80

// FC_StatusFlags3 
#define FC_STATUS3_REDUNDANCE_AKTIVE        0x01
#define FC_STATUS3_BOAT		                0x02
#define FC_STATUS3_REDUNDANCE_ERROR         0x04
#define FC_STATUS3_REDUNDANCE_TEST			0x08

//NC_To_FC_Flags
#define NC_TO_FC_FLYING_RANGE      	0x01
#define NC_TO_FC_EMERGENCY_LANDING 	0x02 // Forces a landing
#define NC_TO_FC_AUTOSTART 			0x04
#define NC_TO_FC_FAILSAFE_LANDING	0x08 // moves Servos into FS-Position
#define NC_TO_FC_SIMULATION_ACTIVE	0x10 // don't start motors

extern volatile unsigned char FC_StatusFlags, FC_StatusFlags2;
extern unsigned char FC_StatusFlags3;
extern void ParameterZuordnung(void);
extern unsigned char GetChannelValue(unsigned char ch); // gives the unsigned value of the channel
extern void ChannelAssingment(void);
extern void StoreNeutralToEeprom(void);

#define Poti1 Poti[0]
#define Poti2 Poti[1]
#define Poti3 Poti[2]
#define Poti4 Poti[3]
#define Poti5 Poti[4]
#define Poti6 Poti[5]
#define Poti7 Poti[6]
#define Poti8 Poti[7]

#define LIMIT_MIN(value, min) {if(value <= min) value = min;}
#define LIMIT_MAX(value, max) {if(value >= max) value = max;}
#define LIMIT_MIN_MAX(value, min, max) {if(value <= min) value = min; else if(value >= max) value = max;}

#define CHK_POTI(b,a) {if(a < 248) b = a; else b = Poti[255 - a];}
#define CHK_POTI_OFF(b,a,off) {if(a < 248) b = a; else b = Poti[255 - a] - off;}
#define CHK_POTI_MM(b,a,min,max) {CHK_POTI(b,a); LIMIT_MIN_MAX(b, min, max);}
#define CHK_POTI_MM_OFF(b,a,min,max,off) {CHK_POTI_OFF(b,a,off); LIMIT_MIN_MAX(b, min, max);}

extern unsigned char Sekunde,Minute;
extern unsigned int BaroExpandActive;
extern long IntegralNick;//,IntegralNick2;
extern long IntegralRoll;//,IntegralRoll2;
//extern int IntegralNick,IntegralNick2;
//extern int IntegralRoll,IntegralRoll2;
extern unsigned char Poti[9];

extern long Mess_IntegralNick;//,Mess_IntegralNick2;
extern long Mess_IntegralRoll;//,Mess_IntegralRoll2;
extern long IntegralAccNick,IntegralAccRoll;
extern long SummeNick,SummeRoll;
extern volatile long Mess_Integral_Hoch;
extern long Integral_Gier,Mess_Integral_Gier,Mess_Integral_Gier2;
extern int  KompassValue;
extern int  KompassSollWert,NC_CompassSetpoint;
extern int  KompassRichtung;
extern char CalculateCompassTimer;
extern unsigned char KompassFusion;
extern unsigned char ControlHeading;
extern int  TrimNick, TrimRoll;
extern long  ErsatzKompass;
extern int   ErsatzKompassInGrad,CompassCorrected; // Kompasswert in Grad
extern long HoehenWert;
extern long SollHoehe;
extern long FromNC_AltitudeSetpoint;
extern unsigned char FromNC_AltitudeSpeed;
extern unsigned char Parameter_HoehenSchalter;      // Wert : 0-250
extern unsigned char CareFree;
extern int MesswertNick,MesswertRoll,MesswertGier;
extern int AdNeutralNick,AdNeutralRoll,AdNeutralGier, Mittelwert_AccNick, Mittelwert_AccRoll;
extern int BoatNeutralNick,BoatNeutralRoll,BoatNeutralGier;
extern unsigned int NeutralAccX, NeutralAccY;
extern unsigned char HoehenReglerAktiv;
extern int NeutralAccZ;
extern signed char NeutralAccZfine;
extern long Umschlag180Nick, Umschlag180Roll;
extern unsigned char Parameter_UserParam1,Parameter_UserParam2,Parameter_UserParam3,Parameter_UserParam4,Parameter_UserParam5,Parameter_UserParam6,Parameter_UserParam7,Parameter_UserParam8;
extern int NaviAccNick,NaviAccRoll,NaviCntAcc;
extern unsigned int modell_fliegt;
extern void MotorRegler(void);
extern void SendMotorData(void);
//void CalibrierMittelwert(void);
//void Mittelwert(void);
extern unsigned char SetNeutral(unsigned char AccAdjustment);  // retuns: "sucess"
extern void Piep(unsigned char Anzahl, unsigned int dauer);
extern void CopyDebugValues(void);
extern unsigned char ACC_AltitudeControl;
extern signed int CosAttitude;	// for projection of hoover gas

extern unsigned char h,m,s;
extern int StickNick,StickRoll,StickGier,StickGas;
extern volatile unsigned char Timeout ;
extern unsigned char CosinusNickWinkel, CosinusRollWinkel;
extern int  DiffNick,DiffRoll;
//extern int  Poti1, Poti2, Poti3, Poti4;
extern volatile unsigned char SenderOkay;
extern int StickNick,StickRoll,StickGier;
extern char MotorenEin;
extern unsigned char CalibrationDone;
extern unsigned char Parameter_Servo3,Parameter_Servo4,Parameter_Servo5;
extern char VarioCharacter;
extern signed int AltitudeSetpointTrimming;
extern signed char WaypointTrimming;
extern int HoverGas;
extern unsigned char Parameter_Luftdruck_D;
//extern unsigned char Parameter_MaxHoehe;
extern unsigned char Parameter_Hoehe_P;
extern unsigned char Parameter_Hoehe_ACC_Wirkung;
extern unsigned char Parameter_KompassWirkung;
extern unsigned char Parameter_Gyro_P;
extern unsigned char Parameter_Gyro_I;
extern unsigned char Parameter_Gier_P;
extern unsigned char Parameter_ServoNickControl;
extern unsigned char Parameter_ServoRollControl;
extern unsigned char Parameter_ServoNickComp;
extern unsigned char Parameter_ServoRollComp;
extern unsigned char Parameter_AchsKopplung1;
extern unsigned char Parameter_AchsKopplung2;
//extern unsigned char Parameter_AchsGegenKopplung1;
extern unsigned char Parameter_J16Bitmask;             // for the J16 Output
extern unsigned char Parameter_J16Timing;              // for the J16 Output
extern unsigned char Parameter_J17Bitmask;             // for the J17 Output
extern unsigned char Parameter_J17Timing;              // for the J17 Output
extern unsigned char Parameter_GlobalConfig;
extern unsigned char Parameter_ExtraConfig;
extern signed char MixerTable[MAX_MOTORS][4];
extern const signed char sintab[31];
extern unsigned char LowVoltageLandingActive;
extern unsigned char LowVoltageHomeActive;
extern unsigned char Parameter_MaximumAltitude;
extern char NeueKompassRichtungMerken;
extern unsigned char ServoFailsafeActive;

#endif //_FC_H

