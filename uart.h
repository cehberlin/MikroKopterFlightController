 #ifndef _UART_H
 #define _UART_H

#define printf_P(format, args...)   _printf_P(&uart_putchar, format , ## args)
#define printf(format, args...)     _printf_P(&uart_putchar, PSTR(format) , ## args)

void BearbeiteRxDaten(void);

extern unsigned char DebugGetAnforderung;
extern unsigned volatile char ReceiverUpdateModeActive;
extern unsigned volatile char UebertragungAbgeschlossen;
extern unsigned volatile char PC_DebugTimeout;
extern unsigned volatile char NeueKoordinateEmpfangen;
extern unsigned volatile char PC_MotortestActive;
extern unsigned char MeineSlaveAdresse;
extern unsigned char PcZugriff;
extern unsigned char RemotePollDisplayLine;
extern unsigned volatile char RxdBuffer[];
extern int Debug_Timer,Kompass_Timer;
extern void UART_Init (void);
extern void  uart_putchar (char c);
//extern void boot_program_page (uint32_t page, uint8_t *buf);
extern void DatenUebertragung(void);
extern void Uart1Init(void);
extern void BearbeiteRxDaten(void);
extern unsigned char MotorTest[16];
struct str_DebugOut
{
 unsigned char Status[2];
 signed int Analog[32];    // Debugwerte
};

extern struct str_DebugOut    DebugOut;

struct str_WinkelOut
{
   signed int  Winkel[2];
 unsigned char UserParameter[2];
 unsigned char CalcState;
 unsigned char Orientation;
};
extern struct str_WinkelOut  WinkelOut;

struct str_Data3D
{
   signed int  Winkel[3]; // nick, roll, compass in 0,1°
   signed char Centroid[3];
   signed char reserve[5];
};
extern struct str_Data3D Data3D;

struct str_ExternControl
{
 unsigned char Digital[2];
 unsigned char RemoteTasten;
 signed char   Nick;
 signed char   Roll;
 signed char   Gier;
 unsigned char Gas;
 signed char   Hight;
 unsigned char free;
 unsigned char Frame;
 unsigned char Config;
};
extern struct str_ExternControl   ExternControl;

// FC hardware errors

// bitmask for UART_VersionInfo_t.HardwareError[0]
#define FC_ERROR0_GYRO_NICK 	0x01
#define FC_ERROR0_GYRO_ROLL 	0x02
#define FC_ERROR0_GYRO_YAW 		0x04
#define FC_ERROR0_ACC_NICK 		0x08
#define FC_ERROR0_ACC_ROLL 		0x10
#define FC_ERROR0_ACC_TOP  		0x20
#define FC_ERROR0_PRESSURE		0x40
#define FC_ERROR0_CAREFREE		0x80
// bitmask for UART_VersionInfo_t.HardwareError[1]
#define FC_ERROR1_I2C   	 	0x01
#define FC_ERROR1_BL_MISSING 	0x02
#define FC_ERROR1_SPI_RX	 	0x04
#define FC_ERROR1_PPM	 		0x08
#define FC_ERROR1_MIXER			0x10
#define FC_ERROR1_RES1			0x20
#define FC_ERROR1_RES2			0x40
#define FC_ERROR1_RES3			0x80


// for FlightCtrl
//VersionInfo.Flags
#define FC_VERSION_FLAG_NC_PRESENT           0x01
// for NaviCtrl
#define NC_VERSION_FLAG_MK3MAG_PRESENT       0x01

struct str_VersionInfo
{
  unsigned char SWMajor;
  unsigned char SWMinor;
  unsigned char ProtoMajor;
  unsigned char reserved1;
  unsigned char SWPatch;
  unsigned char HardwareError[2];
  unsigned char HWMajor;
  unsigned char reserved2;  
  unsigned char Flags;
};

extern struct str_VersionInfo VersionInfo;

//#define USART0_BAUD 9600
//#define USART0_BAUD 14400
//#define USART0_BAUD 28800
//#define USART0_BAUD 38400
#define USART0_BAUD 57600


#endif //_UART_H
