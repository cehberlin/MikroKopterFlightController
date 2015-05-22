#ifndef _HOTTMENU_H
#define _HOTTMENU_H

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))

#define SPEAK_ERR_CALIBARTION  1
#define SPEAK_ERR_RECEICER 	 2
#define SPEAK_ERR_DATABUS  	 3
#define SPEAK_ERR_NAVI	  	 4
#define SPEAK_ERROR		 	 5
#define SPEAK_ERR_COMPASS 	 6
#define SPEAK_ERR_SENSOR 	 7
#define SPEAK_ERR_GPS	 	 8
#define SPEAK_ERR_MOTOR	 	 9
#define SPEAK_MAX_TEMPERAT  10
#define SPEAK_ALTI_REACHED  11
#define SPEAK_WP_REACHED    12
#define SPEAK_NEXT_WP       13
#define SPEAK_LANDING       14
#define SPEAK_GPS_FIX       15
#define SPEAK_UNDERVOLTAGE  16
#define SPEAK_GPS_HOLD      17
#define SPEAK_GPS_HOME      18
#define SPEAK_GPS_OFF       19
#define SPEAK_BEEP          20
#define SPEAK_MIKROKOPTER   21
#define SPEAK_CAPACITY      22
#define SPEAK_CF_OFF        23
#define SPEAK_CALIBRATE     24
#define SPEAK_MAX_RANGE     25
#define SPEAK_MAX_ALTITUD   26

#define SPEAK_MK_OFF	  	38
#define SPEAK_ALTITUDE_ON  	39
#define SPEAK_ALTITUDE_OFF 	40
#define SPEAK_CF_ON      	46
#define SPEAK_SINKING      	47
#define SPEAK_RISING      	48
#define SPEAK_HOLDING      	49
#define SPEAK_GPS_ON      	50
#define SPEAK_FOLLWING     	51
#define SPEAK_STARTING      52

#define MAX_ERR_NUMBER (31+1)
extern const char PROGMEM NC_ERROR_TEXT[MAX_ERR_NUMBER][17];
extern unsigned char NaviData_WaypointIndex, NaviData_WaypointNumber, NaviData_TargetHoldTime;
extern unsigned int NaviData_TargetDistance;

extern unsigned char HottKeyboard,HoTT_RequestedSensor;
extern unsigned char HottUpdate(unsigned char key);
extern unsigned char SpeakHoTT,ShowSettingNameTime;
extern unsigned char ToNC_SpeakHoTT;
extern volatile unsigned char *HoTT_DataPointer;
extern unsigned char MaxBlTempertaure;

extern void CreateHoTT_Menu(void);
extern void LIBFC_HoTT_Putchar(char);
extern void LIBFC_HoTT_Putchar_INV(char); // print Invers
extern void LIBFC_HoTT_Putchar_BLINK(char);
extern void LIBFC_HoTT_SetPos(unsigned char);
extern void LIBFC_HoTT_Clear(void);
extern void NC_Fills_HoTT_Telemety(void);
extern void HoTT_Menu(void);
extern unsigned char HoTT_Telemety(unsigned char);
extern unsigned char HoTT_Waring(void);
extern volatile unsigned char HoTTBlink;
extern void GetHottestBl(void);

typedef struct
{
  unsigned char StartByte;   	// 0x7C
  unsigned char Packet_ID;    	// HOTT_GENERAL_PACKET_ID	
  unsigned char WarnBeep;    	// 3 Anzahl der Töne 0..36
  unsigned char SensorID;       // 4 0xD0
  unsigned char InverseStatus1; // 5  
  unsigned char InverseStatus2; // 6
  unsigned char VoltageCell1;	// 7 208 = 4,16V  (Voltage * 50 = Wert)
  unsigned char VoltageCell2;	// 8 209 = 4,18V
  unsigned char VoltageCell3;	// 9
  unsigned char VoltageCell4;	// 10
  unsigned char VoltageCell5;	// 11
  unsigned char VoltageCell6;	// 12
  unsigned int 	Battery1;		// 13+14 51  = 5,1V
  unsigned int 	Battery2;		// 15+16 51  = 5,1V
  unsigned char Temperature1;	// 17 44 = 24°C, 0 = -20°C
  unsigned char Temperature2;	// 18 44 = 24°C, 0 = -20°C
  unsigned char FuelPercent;    // 19
    signed int  FuelCapacity;   // 20+21
  unsigned int 	Rpm;  			// 22+23
    signed int 	Altitude;		// 24+25
  unsigned int 	m_sec;	 	    // 26+27 3000 = 0
  unsigned char	m_3sec;	 	    // 28 120 = 0
  unsigned int  Current;		// 29+30 1 = 0.1A
  unsigned int 	InputVoltage;	// 31+32 66  = 6,6V
  unsigned int 	Capacity;		// 33+34 1  = 10mAh
  unsigned int  Speed;			// 35+36
  unsigned char LowestCellVoltage; 	// 37
  unsigned char LowestCellNumber;  	// 38
  unsigned int 	Rpm2;  				// 39+40
  unsigned char ErrorNumber;		// 41
  unsigned char Pressure;           // 42  in 0,1bar 20=2,0bar
  unsigned char Version;            // 43
  unsigned char EndByte;   		// 0x7D
} HoTTGeneral_t;

typedef struct
{
  unsigned char StartByte;   	// 0x7C
  unsigned char Packet_ID;    	// HOTT_ELECTRIC_AIR_PACKET_ID	
  unsigned char WarnBeep;    	// Anzahl der Töne 0..36
  unsigned char SensorID;       // 4 0xE0
  unsigned char InverseStatus1; // 5  
  unsigned char InverseStatus2; // 6
  unsigned char VoltageCell1;	// 7 208 = 4,16V  (Voltage * 50 = Wert)
  unsigned char VoltageCell2;	// 209 = 4,18V
  unsigned char VoltageCell3;	// 
  unsigned char VoltageCell4;	// 
  unsigned char VoltageCell5;	// 
  unsigned char VoltageCell6;	// 
  unsigned char VoltageCell7;	// 
  unsigned char VoltageCell8;	// 
  unsigned char VoltageCell9;	// 
  unsigned char VoltageCell10;	// 
  unsigned char VoltageCell11;	// 
  unsigned char VoltageCell12;	// 
  unsigned char VoltageCell13;	// 
  unsigned char VoltageCell14;	// 20 
  unsigned int 	Battery1;		// 21+22 51  = 5,1V
  unsigned int 	Battery2;		// 23+24 51  = 5,1V
  unsigned char Temperature1;	// 25 44 = 24°C, 0 = -20°C
  unsigned char Temperature2;	// 26 44 = 24°C, 0 = -20°C
    signed int 	Altitude;		// 27+28
  unsigned int  Current;		// 29+30 1 = 0.1A
  unsigned int 	InputVoltage;	// 31+32 66  = 6,6V
  unsigned int 	Capacity;		// 33+34 1  = 10mAh
  unsigned int 	m_sec;	 	    // 35+36 30000 = 0
  unsigned char	m_3sec;	 	    // 37 120 = 0
  unsigned int  Rpm;			// 38+39
  unsigned char FlightTimeMinutes; // 40
  unsigned char FlightTimeSeconds; // 41
  unsigned char Speed;			// 42  1=2km 
  unsigned char Version;   		// 43 0x00
  unsigned char EndByte;   		// 0x7D
} ElectricAirPacket_t;


typedef struct
{
  unsigned char StartByte;   	// 0x7C
  unsigned char Packet_ID;    	// 0x89  - Vario ID 
  unsigned char WarnBeep;   //3 	// Anzahl der Töne 0..36
  unsigned char SensorID;        // 0x90
  unsigned char InverseStatus;   
    signed int Altitude;	//6+7    // 500 = 0m
    signed int MaxAltitude;	//8+9    // 500 = 0m
	signed int MinAltitude;	//10+11   // 500 = 0m
  unsigned int m_sec;	 	//12+13    // 3000 = 0
  unsigned int	m_3sec;	 	//14+15    
  unsigned int  m_10sec;	//26+17
  char 		    Text[21];   //18-38
  char 		    FreeCharacters[3]; // 39-41
  unsigned char NullByte;   // 42 0x00
  unsigned char Version;   	// 43
  unsigned char EndByte;   	// 0x7D
} VarioPacket_t;

typedef struct
{
  unsigned char StartByte;  //0 	// 0x7C
  unsigned char Packet_ID;  //1  	// 0x8A  - GPS ID 
  unsigned char WarnBeep;   //2 	// Anzahl der Töne 0..36
  unsigned char SensorID;       // 4 0xA0
  unsigned char InverseStatus1; // 5  
  unsigned char InverseStatus2; // 6
  unsigned char Heading;	//7  	// 1 = 2°
  unsigned int Speed;		//8+9   // in km/h
  unsigned char Lat_North;	//10    
  unsigned char Lat_G;	    //11
  unsigned char Lat_M;	    //12
  unsigned char Lat_Sek1;	//13    
  unsigned char Lat_Sek2;	//14    
  unsigned char Lon_East;	//15    
  unsigned char Lon_G;	    //16
  unsigned char Lon_M;	    //17
  unsigned char Lon_Sek1;	//18    
  unsigned char Lon_Sek2;	//19    
  unsigned int Distance;	//20+21	   // 9000 = 0m
    signed int Altitude;	//22+23    // 500 = 0m
  unsigned int m_sec;	 	//24+25    // 3000 = 0
  unsigned char	m_3sec;	 	//26 120 = 0
  unsigned char NumOfSats;	//27
  unsigned char SatFix;     //28
  unsigned char HomeDirection; // 29
  unsigned char AngleX;		  // 30	
  unsigned char AngleY;       // 31
  unsigned char AngleZ;       // 32
    signed int GyroX;         //33+34
    signed int GyroY;         //35+36
    signed int GyroZ;         //37+38
  unsigned char Vibration;    // 39
  char 		    FreeCharacters[3]; // 40-42
  unsigned char Version;   	// 43
  unsigned char EndByte;   		// 0x7D
} GPSPacket_t;

typedef struct
{
  unsigned char StartByte;   	// 0x7B
  unsigned char Packet_ID;    	// 
  unsigned char WarnBeep;    	// Anzahl der Töne 0..36
  char Text[8*21];
  unsigned char EndByte;   		// 0x7D
} ASCIIPacket_t;



extern GPSPacket_t GPSPacket;
extern VarioPacket_t VarioPacket;
extern ASCIIPacket_t ASCIIPacket;
extern ElectricAirPacket_t ElectricAirPacket;
extern HoTTGeneral_t HoTTGeneral;

#define HOTT_VARIO_PACKET_ID		0x89
#define HOTT_GPS_PACKET_ID			0x8A
#define HOTT_ELECTRIC_AIR_PACKET_ID	0x8E
#define HOTT_GENERAL_PACKET_ID		0x8D

#endif 
#endif

