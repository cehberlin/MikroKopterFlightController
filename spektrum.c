/*#######################################################################################
Decodieren eines RC Summen Signals oder Spektrum Empfänger-Satellit
#######################################################################################*/

#include "spektrum.h"
#include "main.h"

unsigned char SpektrumTimer = 0;

// Achtung: RECEIVER_SPEKTRUM_DX7EXP oder RECEIVER_SPEKTRUM_DX8EXP wird in der main.h gesetzt
#if defined (RECEIVER_SPEKTRUM_DX7EXP) || defined (RECEIVER_SPEKTRUM_DX8EXP)
unsigned char s_excnt = 0;                   // Bitcounter for Spektrum-Expander
unsigned char s_exparity = 0;                // Parity Bit for Spektrum-Expander
signed char s_exdata[11];                    // Data for Spektrum-Expander

void s_update(unsigned char channel, signed int value)  // Channel-Diff numbercrunching and finally assign new stickvalue to PPM_in
{
        if(SenderOkay >= 180) PPM_diff[channel] = ((value - PPM_in[channel]) / 3) * 3;
        else PPM_diff[channel] = 0;
        PPM_in[channel] = value;
}
#endif

//############################################################################
// USART1 initialisation from killagreg
void SpektrumUartInit(void)
//############################################################################
    {
        // -- Start of USART1 initialisation for Spekturm seriell-mode
        // USART1 Control and Status Register A, B, C and baud rate register
        uint8_t sreg = SREG;
       
        uint16_t ubrr = (uint16_t) ((uint32_t) SYSCLK/(8 * 115200) - 1);
       
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
        // no parity
        UCSR1C &= ~(1 << UPM11);
        UCSR1C &= ~(1 << UPM10);
        // 1 stop bit
        UCSR1C &= ~(1 << USBS1);
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
   
        SREG = sreg;
  return;
 }

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Rainer Walther
// + RC-routines from original MK rc.c (c) H&I
// + Useful infos from Walter: http://www.rcgroups.com/forums/showthread.php?t=714299&page=2
// + only for non-profit use
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
// 20080808 rw Modified for Spektrum AR6100 (PPM)
// 20080823 rw Add Spektrum satellite receiver on USART1 (644P only)
// 20081213 rw Add support for Spektrum DS9 Air-Tx-Module (9 channels)
//             Replace AR6100-coding with original composit-signal routines
//
// ---
// Entweder Summensignal ODER Spektrum-Receiver anschließen. Nicht beides gleichzeitig betreiben!
// Binding is not implemented. Bind with external Receiver.
// Servo output J3, J4, J5 not serviced
//
// Anschuß Spektrum Receiver
//              Orange:         3V von der FC (keinesfalls an 5V anschließen!)
//              Schwarz:        GND
//              Grau:           RXD1 (Pin 3) auf 10-Pol FC-Stecker
//
// ---
// Satellite-Reciever connected on USART1:
//
// DX7/DX6i: One data-frame at 115200 baud every 22ms.
// DX7se:    One data-frame at 115200 baud every 11ms.
//              byte1:  unknown
//      byte2:  unknown
//      byte3:  and byte4:  channel data        (FLT-Mode)
//      byte5:  and byte6:  channel data        (Roll)
//      byte7:  and byte8:  channel data        (Nick)
//      byte9:  and byte10: channel data        (Gier)
//      byte11: and byte12: channel data        (Gear Switch)
//      byte13: and byte14: channel data        (Gas)
//      byte15: and byte16: channel data        (AUX2)
//
// DS9 (9 Channel): One data-frame at 115200 baud every 11ms, alternating frame 1/2 for CH1-7 / CH8-9
//  1st Frame:
//              byte1:  unknown
//      byte2:  unknown
//              byte3:  and byte4:  channel data
//      byte5:  and byte6:  channel data
//      byte7:  and byte8:  channel data
//      byte9:  and byte10: channel data
//      byte11: and byte12: channel data
//      byte13: and byte14: channel data
//      byte15: and byte16: channel data
//  2nd Frame:
//              byte1:  unknown
//      byte2:  unknown
//              byte3:  and byte4:  channel data
//      byte5:  and byte6:  channel data
//      byte7:  and byte8:  0xffff
//      byte9:  and byte10: 0xffff
//      byte11: and byte12: 0xffff
//      byte13: and byte14: 0xffff
//      byte15: and byte16: 0xffff
//
// Each channel data (16 bit= 2byte, first msb, second lsb) is arranged as:
//
// Bits: F 0 C3 C2 C1 C0 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
//
// 0 means a '0' bit
// F: 1 = indicates beginning of 2nd frame for CH8-9 (DS9 only)
// C3 to C0 is the channel number. 0 to 9 (4 bit, as assigned in the transmitter)
// D9 to D0 is the channel data (10 bit) 0xaa..0x200..0x356 for 100% transmitter-travel
//
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MIN_FRAMEGAP 68  // 7ms
#define MAX_BYTEGAP  3   // 310us


//############################################################################
// Wird im UART-Interrupt aufgerufen
//############################################################################
void SpektrumParser(unsigned char c)
{
    static unsigned char Sync=0, FrameCnt=0, ByteHigh=0, ReSync=1, Frame2=0;
        unsigned int Channel, index = 0;
        signed int signal = 0, tmp;
        int bCheckDelay;
//      c = UDR1; // get data byte
        if(ReSync == 1)
            {
                // wait for beginning of new frame
                ReSync = 0;
                SpektrumTimer = MIN_FRAMEGAP;
                FrameCnt = 0;
                Sync = 0;
                ByteHigh = 0;
                }
  else
  {
        if(!SpektrumTimer) bCheckDelay = 1; else bCheckDelay = 0;//CheckDelay(FrameTimer);
        if ( Sync == 0 )
            {
                if(bCheckDelay)
                    {
                        // nach einer Pause von mind. 7ms erstes Sync-Character gefunden
                        // Zeichen ignorieren, da Bedeutung unbekannt
                        Sync = 1;
                        FrameCnt ++;
                    SpektrumTimer = MAX_BYTEGAP;
                        }
                else
                        {
                        // Zeichen kam vor Ablauf der 7ms Sync-Pause
                        // warten auf erstes Sync-Zeichen
                        SpektrumTimer = MIN_FRAMEGAP;
                    FrameCnt = 0;
                    Sync = 0;
                    ByteHigh = 0;
                        }
                }
        else if((Sync == 1) && !bCheckDelay)
            {
                // zweites Sync-Character ignorieren, Bedeutung unbekannt
                Sync = 2;
                FrameCnt ++;
                SpektrumTimer = MAX_BYTEGAP;
                }
        else if((Sync == 2) && !bCheckDelay)
            {
                SpektrumTimer = MAX_BYTEGAP;
                // Datenbyte high
                ByteHigh = c;
                if (FrameCnt == 2)
                    {
                    // is 1st Byte of Channel-data
                        // Frame 1 with Channel 1-7 comming next
                        Frame2 = 0;
                        if(ByteHigh & 0x80)
                            {
                                // DS9: Frame 2 with Channel 8-9 comming next
                                Frame2 = 1;
                                }
                        }
                Sync = 3;
                FrameCnt ++;
                }
        else if((Sync == 3) && !bCheckDelay)
            {
                // Datenbyte low
                // High-Byte for next channel comes next
                SpektrumTimer = MAX_BYTEGAP;
                Sync = 2;
                FrameCnt ++;
                Channel = ((unsigned int)ByteHigh << 8) | c;
                if(EE_Parameter.Receiver == RECEIVER_SPEKTRUM)
                {
                        signal = Channel & 0x3ff;
                        signal -= 0x200;                // Offset, range 0x000..0x3ff?
                        signal = signal/3;              // scaling to fit PPM resolution
                        index = (ByteHigh >> 2) & 0x0f;
                }
                else  
                if(EE_Parameter.Receiver == RECEIVER_SPEKTRUM_HI_RES)
                {
                        signal = Channel & 0x7ff;
                        signal -= 0x400;                // Offset, range 0x000..0x7ff?
                        signal = signal/6;              // scaling to fit PPM resolution
                        index = (ByteHigh >> 3) & 0x0f;
                }      
                else  
                //if(EE_Parameter.Receiver == RECEIVER_SPEKTRUM_LOW_RES)
                {
                        signal = Channel & 0x3ff;
                        signal -= 360;          // Offset, range 0x000..0x3ff?
                        signal = signal/2;              // scaling to fit PPM resolution
                        index = (ByteHigh >> 2) & 0x0f;
                }

                index++;
                if(index < 13)
                {
                // Stabiles Signal
#if defined (RECEIVER_SPEKTRUM_DX7EXP) || defined (RECEIVER_SPEKTRUM_DX8EXP)
                                        if (index == 2) index = 4;                                                              // Analog channel reassigment (2 <-> 4) for logical numbering (1,2,3,4)
                                        else if (index == 4) index = 2;
#endif
                                if(abs(signal - PPM_in[index]) < 6)
                                   {
                                                                         if(EE_Parameter.FailsafeChannel == 0 || PPM_in[EE_Parameter.FailsafeChannel] < 100)  // forces Failsafe if the receiver doesn't have 'signal loss' on Failsafe
                                                                         {
                                        if(SenderOkay < 200) SenderOkay += 10;
                                         else
                                         {
                                                SenderOkay = 200;
                                                TIMSK1 &= ~_BV(ICIE1); // disable PPM-Input
                                         }
                                                                         }     
                                                                   }
                                tmp = (3 * (PPM_in[index]) + signal) / 4;
                                if(tmp > signal+1) tmp--; else
                                if(tmp < signal-1) tmp++;
                               
#ifdef RECEIVER_SPEKTRUM_DX7EXP
                                if(index == 6)                                                                                  // FLIGHT-MODE - The channel used for our data uplink
                                {
                                        if (signal > 100)                                                                       // SYNC received
                                        {
                                                if (s_exdata[s_excnt] == 125) s_exparity = ~s_exparity;                         // Bit = 1 -> Re-Invert parity bit
                                                if ((s_excnt == 6 && ((s_exparity != 0 && s_exdata[s_excnt] == -125) || (s_exparity == 0 && s_exdata[s_excnt] == 125))) || (s_excnt == 9 && ((s_exparity == 0 && s_exdata[s_excnt] == -125) || (s_exparity != 0 && s_exdata[s_excnt] == 125)))) // Parity check
                                                {
                                                        if (s_exdata[1] == 125 && s_exdata[2] == -125) s_update(5,-125);        // Reconstruct tripole Flight-Mode value (CH5)
                                                        else if (s_exdata[1] == -125 && s_exdata[2] == -125) s_update(5,0);     // Reconstruct tripole Flight-Mode value (CH5)
                                                        else if (s_exdata[1] == -125 && s_exdata[2] == 125) s_update(5,125);    // Reconstruct tripole Flight-Mode value (CH5)
                                                        s_update(6,s_exdata[3]);                                                // Elevator (CH6)
                                                        s_update(11,s_exdata[4]);                                               // Aileron (CH11)
                                                        s_update(12,s_exdata[5]);                                               // Rudder (CH12)

                                                        if (s_excnt == 9)                                                       // New Mode (12 Channels)
                                                        {
                                                                if (s_exdata[7] == 125) s_update(8,PPM_in[8]+5);                // Hover Pitch UP (CH8)
                                                                if (s_exdata[8] == 125) s_update(8,PPM_in[8]-5);                // Hover Pitch DN (CH8)
                                                                if (PPM_in[8] < -125) PPM_in[8] = -125;                         // Range-Limit
                                                                else if (PPM_in[8] > 125) PPM_in[8] = 125;                      // Range-Limit
                                                                s_update(10,s_exdata[6]);                                       // AUX2 (CH10)
                                                        }
                                                }

                                                s_excnt = 0;                                                                    // Reset bitcounter
                                                s_exparity = 0;                                                                 // Reset parity bit
                                        }

                                        if (signal < 10) s_exdata[++s_excnt] = -125;                                            // Bit = 0 -> value = -125 (min)
                                        if (s_excnt == 10) s_excnt = 0;                                                         // Overflow protection
                                        if (signal < -100)
                                        {
                                                s_exdata[s_excnt] = 125;                                                        // Bit = 1 -> value = 125 (max)
                                                s_exparity = ~s_exparity;                                                       // Bit = 1 -> Invert parity bit
                                        }

                                }

                                if (index < 5 ) s_update(index,tmp);                                                            // Update normal potis (CH1-4)
                                else if (index == 5) s_update(7,signal);                                                        // Gear (CH7)
                                else if (index == 7) s_update(9,signal);                                                        // Hover Throttle (CH9)

#elif defined RECEIVER_SPEKTRUM_DX8EXP
                                if(index == 6)                                                                                  // FLIGHT-MODE - The channel used for our data uplink
                                {
                                        if (signal > 100)                                                                       // SYNC received
                                        {
                                                if (s_exdata[s_excnt] == 125) s_exparity = ~s_exparity; // Bit = 1 -> Re-Invert parity bit
                                                if (s_excnt == 9 && ((s_exparity == 0 && s_exdata[s_excnt] == -125) || (s_exparity != 0 && s_exdata[s_excnt] == 125)))  // Parity check
                                                {
                                                        if (s_exdata[1] == 125 && s_exdata[2] == -125) s_update(5,-125);        // Reconstruct tripole Flight-Mode value (CH5)
                                                        else if (s_exdata[1] == -125 && s_exdata[2] == -125) s_update(5,0);     // Reconstruct tripole Flight-Mode value (CH5)
                                                        else if (s_exdata[1] == -125 && s_exdata[2] == 125) s_update(5,125);    // Reconstruct tripole Flight-Mode value (CH5)

                                                        if (s_exdata[3] == 125 && s_exdata[6] == -125) s_update(6,125);         // Reconstruct tripole Elev D/R value (CH6)
                                                        else if (s_exdata[3] == -125 && s_exdata[6] == -125) s_update(6,0);     // Reconstruct tripole Elev D/R value (CH6)
                                                        else if (s_exdata[3] == -125 && s_exdata[6] == 125) s_update(6,-125);   // Reconstruct tripole Elev D/R value (CH6)


                                                        if (s_exdata[7] == 125 && s_exdata[8] == -125) s_update(9,-125);        // Reconstruct tripole AIL D/R value (CH9)
                                                        else if (s_exdata[7] == -125 && s_exdata[8] == -125) s_update(9,0);     // Reconstruct tripole AIL D/R value (CH9)
                                                        else if (s_exdata[7] == -125 && s_exdata[8] == 125) s_update(9,125);    // Reconstruct tripole AIL D/R value (CH9)

                                                        s_update(10,s_exdata[5]);                                               // Gear (CH10)
                                                        s_update(12,s_exdata[4]);                                               // Mix (CH12)
                                                }

                                                s_excnt = 0;                                                                    // Reset bitcounter
                                                s_exparity = 0;                                                                 // Reset parity bit
                                        }

                                        if (signal < 10) s_exdata[++s_excnt] = -125;                                            // Bit = 0 -> value = -125 (min)
                                        if (s_excnt == 10) s_excnt = 0;                                                         // Overflow protection
                                        if (signal < -100)
                                        {
                                                s_exdata[s_excnt] = 125;                                                        // Bit = 1 -> value = 125 (max)
                                                s_exparity = ~s_exparity;                                                       // Bit = 1 -> Invert parity bit
                                        }

                                }

                                if (index < 5 ) s_update(index,tmp);                                                            // Update normal potis (CH1-4)
                                else if (index == 7) s_update(7,signal);                                                        // R Trim (CH7)
                                else if (index == 5) s_update(8,signal);                                                        // AUX2 (CH8)
                                else if (index == 8) s_update(11,signal);                                                       // AUX3 (CH11)

#else
                                if(SenderOkay >= 180) PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;
                                else PPM_diff[index] = 0;
                                PPM_in[index] = tmp;
#endif
                        }
        else if(index > 17) ReSync = 1; // hier stimmt was nicht: neu synchronisieren
                }
        else
                {
                // hier stimmt was nicht: neu synchronisieren
                ReSync = 1;
                FrameCnt = 0;
                Frame2 = 0;
                // new frame next, nach fruehestens 7ms erwartet
                SpektrumTimer = MIN_FRAMEGAP;
                }

        // 16 Bytes eingetroffen -> Komplett
        if(FrameCnt >= 16)
            {
                // Frame complete
                if(Frame2 == 0)
                        {
                        // Null bedeutet: Neue Daten
                        // nur beim ersten Frame (CH 0-7) setzen
                        if(!ReSync) NewPpmData = 0;
                        }
                FrameCnt = 0;
                Frame2 = 0;
                Sync = 0;
                SpektrumTimer = MIN_FRAMEGAP;
                }
   }
}
 

