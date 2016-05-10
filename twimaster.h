#ifndef _I2C_MASTER_H
#define _I2C_MASTER_H

#include <inttypes.h>

#define TWI_STATE_MOTOR_TX 			0
#define TWI_STATE_MOTOR_RX 			5
//#define TWI_STATE_GYRO_OFFSET_TX	18

extern volatile uint8_t twi_state, ReadBlSize;
extern volatile uint8_t motor_write;
extern volatile uint8_t motor_read;
extern volatile uint8_t I2C_TransferActive;
extern uint8_t Max_I2C_Packets;
extern uint8_t MissingMotor;

#define MAX_MOTORS	16
#define MOTOR_STATE_PRESENT_MASK		0x80
#define MOTOR_STATE_ERROR_MASK			0x7F

//Motor[x].Version
#define MOTOR_STATE_NEW_PROTOCOL_MASK 	0x01
#define MOTOR_STATE_FAST_MODE           0x02
#define MOTOR_STATE_BL30                0x04   // extended Current measurement -> 200 = 20A    201 = 21A    255 = 75A (20+55)

#define BLFLAG_TX_COMPLETE		0x01
#define BLFLAG_READ_VERSION 	0x02

extern volatile uint8_t BLFlags;

#define BL_READMODE_STATUS  0
#define BL_READMODE_CONFIG	16

typedef struct
{
	uint8_t Version;			// the version of the BL (0 = old)
	uint8_t SetPoint; 			// written by attitude controller
	uint8_t SetPointLowerBits;	// for higher Resolution of new BLs
	uint8_t State;    			// 7 bit for I2C error counter, highest bit indicates if motor is present
	uint8_t ReadMode;			// select data to read
	// the following bytes must be exactly in that order!
	uint8_t Current;  			// in 0.1 A steps, read back from BL
	uint8_t MaxPWM;   			// read back from BL -> is less than 255 if BL is in current limit, not running (250) or starting (40)
	uint8_t Temperature;		// old BL-Ctrl will return a 255 here, the new version the temp. in °C
	uint8_t RPM;				// Raw value for RPM
	uint8_t reserved1;			// Voltage (BL3) or mAh (BL2)
	uint8_t Voltage;			// in 0.1V (BL3 is limited to 255, BL2 is only low-byte)
	uint8_t SlaveI2cError;		// BL2 & BL3
	uint8_t VersionMajor;		// BL2 & BL3
	uint8_t VersionMinor;		// BL2 & BL3
	uint8_t NotReadyCnt; 		// Counts up is the Motor is not ready during flight -> MotorRestart etc.
} __attribute__((packed)) MotorData_t;

extern MotorData_t Motor[MAX_MOTORS];

// BitSate
#define BL_BIT_STATE_I2C_OK 	0x01
#define BL_BIT_STATE_I2C_VALUE 	0x02
#define BL_BIT_STATE_I2C_BAD	0x04
#define BL_BIT_STATE_PPM_OK	 	0x08
#define BL_BIT_STATE_MOTOR_RUN 	0x10

typedef struct
{
	unsigned char BitSate; 
	unsigned char Current; 
	unsigned char State;  
	unsigned char TemperatureInDeg;	  
	unsigned char Voltage80;	  
} __attribute__((packed)) RedundantBl_t;
extern RedundantBl_t RedundantMotor[MAX_MOTORS];

#define BLCONFIG_REVISION 2

#define MASK_SET_PWM_SCALING		0x01
#define MASK_SET_CURRENT_LIMIT		0x02
#define MASK_SET_TEMP_LIMIT			0x04
#define MASK_SET_CURRENT_SCALING	0x08
#define MASK_SET_BITCONFIG			0x10
#define MASK_SET_STARTPWM			0x20
#define MASK_SET_DEFAULT_PARAMS		0x40
#define MASK_SET_SAVE_EEPROM	 	0x80

#define BITCONF_REVERSE_ROTATION 0x01
#define BITCONF_STARTGAS1 0x02
#define BITCONF_STARTGAS2 0x04
#define BITCONF_STARTGAS3 0x08
#define BITCONF_RES4 0x10
#define BITCONF_RES5 0x20
#define BITCONF_RES6 0x40
#define BITCONF_RES7 0x80

typedef struct
{
	uint8_t Revision;			// must be BL_REVISION
	uint8_t SetMask;			// settings mask
	uint8_t PwmScaling;			// maximum value of control pwm, acts like a thrust limit
	uint8_t CurrentLimit;		// current limit in A
	uint8_t TempLimit;			// in °C
	uint8_t CurrentScaling;		// scaling factor for current measurement
	uint8_t BitConfig;			// see defines above
	uint8_t crc;				// checksum
}  __attribute__((packed)) BLConfig_t;

extern BLConfig_t BLConfig;

extern volatile uint16_t I2CTimeout;

void I2C_Init(char); // Initialize I2C
#define I2C_Start(start_state) {twi_state = start_state; BLFlags &= ~BLFLAG_TX_COMPLETE; TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT) | (1<<TWIE);}
#define I2C_Stop(start_state)  {twi_state = start_state; TWCR = (1<<TWEN) | (1<<TWSTO) | (1<<TWINT);}
void I2C_Reset(void); // Reset I2C

#define BLCONFIG_SUCCESS			     0
#define BLCONFIG_ERR_MOTOR_RUNNING       1
#define BLCONFIG_ERR_MOTOR_NOT_EXIST     2
#define BLCONFIG_ERR_HW_NOT_COMPATIBLE   3
#define BLCONFIG_ERR_SW_NOT_COMPATIBLE   4
#define BLCONFIG_ERR_CHECKSUM            5
#define BLCONFIG_ERR_READ_NOT_POSSIBLE   6

uint8_t I2C_WriteBLConfig(uint8_t motor);
uint8_t I2C_ReadBLConfig(uint8_t motor);

#endif
