#include <avr/io.h>

#define J16_ON          PORTC |=  (1<<PORTC2)
#define J16_OFF         PORTC &= ~(1<<PORTC2)
#define J16_TOGGLE      PORTC ^=  (1<<PORTC2)
#define J17_ON          PORTC |=  (1<<PORTC3)
#define J17_OFF         PORTC &= ~(1<<PORTC3)
#define J17_TOGGLE      PORTC ^=  (1<<PORTC3)

extern void LED_Init(void);
extern void LED_Update(void);
