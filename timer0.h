
#define TIMER_TEILER        CK8
#define TIMER_RELOAD_VALUE  250
#define HEF4017Reset_ON     PORTC |=  (1<<PORTC6)
#define HEF4017Reset_OFF    PORTC &= ~(1<<PORTC6)

void Timer_Init(void);
void TIMER2_Init(void);
void Delay_ms(unsigned int);
void Delay_ms_Mess(unsigned int);
unsigned int SetDelay (unsigned int t);
char CheckDelay (unsigned int t);
void CalculateServo(void);
void CalcNickServoValue(void);

extern volatile unsigned int CountMilliseconds;
extern volatile unsigned char UpdateMotor;
extern volatile unsigned int beeptime;
extern volatile unsigned int cntKompass;
extern unsigned int BeepMuster;
extern volatile unsigned char BytegapSPI, ServoActive, CalculateServoSignals;
extern volatile int16_t	ServoNickValue;
extern volatile int16_t	ServoRollValue;
extern signed int NickServoValue;
extern unsigned char JustMK3MagConnected;
