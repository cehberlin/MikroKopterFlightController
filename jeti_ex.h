#ifndef _JETI_EX_H
#define _JETI_EX_H


#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
extern void BuildJeti_Vario(void);

// define here how many Jeti EX parameters should be transmitted (max. = 15)
//
#define JETI_EX_PARAMETER_COUNT 	15
//
// -------------------------------------------------------------------------

extern const char PROGMEM JETI_CODE[53];
	 
typedef struct
{
   char Label[10];
   char Unit[3];
   unsigned char DataType;
   long Value;
   unsigned char DecimalPointPos;
}  JetiExPacket_t;
  
   
extern JetiExPacket_t JetiExData[];
extern void JetiEX_Update(void);



#if (JETI_EX_PARAMETER_COUNT > 15) 
	#error "ERROR: Too many Jeti EX parameters (max. allowed 15)"
#endif

 
#endif
#endif //_JETI_EX_H
