/*#######################################################################################
Decodes the sbus protocol
#######################################################################################*/
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

#include "sbus.h"
#include "main.h"

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
unsigned char NewSBusData = 0, sBusBuffer[25];

//############################################################################
// USART1 initialisation from killagreg
void SbusUartInit(void)
//############################################################################
    {
        // -- Start of USART1 initialisation for Spekturm seriell-mode
        // USART1 Control and Status Register A, B, C and baud rate register
        uint8_t sreg = SREG;
       
        uint16_t ubrr = (uint16_t) ((uint32_t) SYSCLK/(8 * 100000) - 1);
       
        // disable all interrupts before reconfiguration
        cli();
        // disable RX-Interrupt
        UCSR1B &= ~(1 << RXCIE1);
        // disable TX-Interrupt
        UCSR1B &= ~(1 << TXCIE1);
        // disable DRE-Interrupt
        UCSR1B &= ~(1 << UDRIE1);
/*
        // set direction of RXD1 and TXD1 pins
        // set RXD1 (PD2) as an input pin
        PORTD |= (1 << PORTD2);
        DDRD &= ~(1 << DDD2);
        // set TXD1 (PD3) as an output pin
        PORTD |= (1 << PORTD3);
        DDRD  |= (1 << DDD3);
*/       
        // USART0 Baud Rate Register
        // set clock divider
        UBRR1H = (uint8_t)(ubrr>>8);
        UBRR1L = (uint8_t)ubrr;
        // enable double speed operation
        UCSR1A |= (1 << U2X1);
        // enable receiver and transmitter
        //UCSR1B = (1<<RXEN1)|(1<<TXEN1);

        UCSR1B = (1<<RXEN1);
        // set asynchronous mode
        UCSR1C &= ~(1 << UMSEL11);
        UCSR1C &= ~(1 << UMSEL10);
        // parity
        UCSR1C |=  (1 << UPM11);   // even
        UCSR1C &= ~(1 << UPM10);
        //  stop bit
        UCSR1C |= (1 << USBS1);    // two
        // 8-bit
        UCSR1B &= ~(1 << UCSZ12);
        UCSR1C |=  (1 << UCSZ11);
        UCSR1C |=  (1 << UCSZ10);
        // flush receive buffer explicit
        while(UCSR1A & (1<<RXC1)) UDR1;
        // enable RX-interrupts at the end
        UCSR1B |= (1 << RXCIE1);
        // -- End of USART1 initialisation
        // restore global interrupt flags
        sBusBuffer[23] |= 4; // This Bit contains the 'Signal loss'
        SREG = sreg;
  return;
 }

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MIN_FRAMEGAP 68  // 7ms
#define MAX_BYTEGAP  3   // 310us

//############################################################################
// Is called by the uart RX interrupt
//############################################################################
void SbusParser(unsigned char udr)
{
 static unsigned char  ptr = 0;
  if(!SpektrumTimer && udr == 0x0f)  // wait for the start
   {
	ptr = 0; 
    SpektrumTimer = 80; // 8ms gap
 	}
	else 
	{
     if(++ptr == 24)              	// last byte
	  {
		NewSBusData = 1;
	  }
	else 
	if(ptr > 24) ptr = 25;
	else 
	{
	 sBusBuffer[ptr] = udr; // collect all bytes
	}
   }
}

void ProcessSBus(void)
{
 static unsigned char load = 0;
 unsigned char bitmask8 = 1, sbyte = 2, i, index = 1, process;
 unsigned int bitmask11 = 256;
 signed int signal = 0,tmp;
 
 if(!(sBusBuffer[23] & 4)) 	// This Bit contains the 'Signal loss'
	   {
	    TIMSK1 &= ~_BV(ICIE1); // disable PPM-Input
 		if(EE_Parameter.FailsafeChannel == 0 || PPM_in[EE_Parameter.FailsafeChannel] < 100)  // forces Failsafe if the receiver doesn't have 'signal loss' on Failsafe
		  {
		    if(SenderOkay < 200) SenderOkay += 20; else SenderOkay = 200;
		  } 
		signal = sBusBuffer[1];
        if(!load--) { process = (12*11 - 8); load = 2;} else process = (4*11 - 8);  // lowers the processor load 
		for(i = 0; i < process; i++)  // collect the single bits
		{
			if(sBusBuffer[sbyte] & bitmask8) signal |= bitmask11;
			bitmask8 *= 2; 
			if(!bitmask8)
			{
			 bitmask8 = 1;
 			 sbyte++;
			}
			bitmask11 *= 2;
		    if(bitmask11 == 2048)
		    {
			 bitmask11 = 1;
			 signal = (signal-1024) / 5; // the resolution is higher than required
                tmp = (3 * (PPM_in[index]) + signal) / 4; 
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 195)  PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;
                else PPM_diff[index] = 0;
                PPM_in[index] = tmp;
			 signal = 0;
			 index++; // next channel
			}
		}
	    NewPpmData = 0;  // Null bedeutet: Neue Daten
	   }
 NewSBusData = 0;
}

#endif
