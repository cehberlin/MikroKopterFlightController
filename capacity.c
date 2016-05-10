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

#include "capacity.h"
#include "twimaster.h"
#include "main.h"
#include "timer0.h"
#include "analog.h"

//#define CAPACITY_UPDATE_INTERVAL 10 // 10 ms
#define CAPACITY_UPDATE_INTERVAL 50 // 50 ms  = 20Hz
#define FC_OFFSET_CURRENT 5  // calculate with a current of 0.5A
#define BL_OFFSET_CURRENT 2  // calculate with a current of 0.2A

// global varialbles
unsigned short update_timer =  0;
Capacity_t Capacity;

// initialize capacity calculation
void Capacity_Init(void)
{
	Capacity.ActualCurrent = 0;
	Capacity.UsedCapacity = 0;
	Capacity.ActualPower = 0;
	Capacity.MinOfMaxPWM = 0;
	update_timer = SetDelay(CAPACITY_UPDATE_INTERVAL);
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + extended Current measurement -> 200 = 20A    201 = 21A    255 = 75A (20+55)
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned int BL3_Current(unsigned char who) // in 0,1A
{
 if(Motor[who].Current == 255) return(0); // invalid
 if(Motor[who].Current <= 200) return(Motor[who].Current);
 else
 {
  if(Motor[who].Version & MOTOR_STATE_BL30) return(200 + 10 * ((unsigned int)Motor[who].Current-200));
  else return(Motor[who].Current);
 }
}

// called in main loop at a regular interval
void Capacity_Update(void)
{
	unsigned short Current, SetSum; // max value will be 255 * 12 = 3060
	static unsigned short SubCounter = 0;
	static unsigned short CurrentOffset = 0;
	static unsigned long SumCurrentOffset = 0;
	unsigned char i, NumOfMotors, MinOfMaxPWM;

	if(CheckDelay(update_timer))
	{
		update_timer += CAPACITY_UPDATE_INTERVAL; // do not use SetDelay to avoid timing leaks
		// determine sum of all present BL currents and setpoints
		Current = 0;
		SetSum = 0;
		NumOfMotors = 0;
		MinOfMaxPWM = 255;
		if(Capacity.MinOfMaxPWM == 254) 		FC_StatusFlags3 |= FC_STATUS3_REDUNDANCE_AKTIVE;
//		else if(Capacity.MinOfMaxPWM == 255) 	FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_AKTIVE;
		
		for(i = 0; i < MAX_MOTORS; i++)
		{
			if(Motor[i].State & MOTOR_STATE_PRESENT_MASK/* && Mixer.Motor[i][MIX_GAS]*/)
			{
				NumOfMotors++;
				if(Motor[i].Current > 200)
				 {
				  Current += BL3_Current(i); // extended Current measurement -> 200 = 20A    201 = 21A    255 = 75A (20+55)
				 }
				 else Current += (unsigned int)(Motor[i].Current);
				SetSum +=  (unsigned int)(Motor[i].SetPoint);
				if(Motor[i].MaxPWM <= MinOfMaxPWM) MinOfMaxPWM = Motor[i].MaxPWM; 
				else 
				if(Motor[i].MaxPWM == 255) FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_AKTIVE;
			}
		}
		Capacity.MinOfMaxPWM = MinOfMaxPWM;
		if(SetSum == 0) // if all setpoints are 0
		{ // determine offsets of motor currents
			#define CURRENT_AVERAGE 8  // 8bit = 256 * 10 ms = 2.56s average time
			CurrentOffset = (unsigned int)(SumCurrentOffset>>CURRENT_AVERAGE);
			SumCurrentOffset -= CurrentOffset;
			SumCurrentOffset += Current;
			// after averaging set current to static offset
			Current = FC_OFFSET_CURRENT;
			FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_AKTIVE;	
		}
		else // some motors are running, includes also motor test condition, where "MotorRunning" is false
		{   // subtract offset
			if(Current > CurrentOffset) Current -= CurrentOffset;
			else Current = 0;
			// add the FC and BL Offsets
			Current += FC_OFFSET_CURRENT + NumOfMotors * BL_OFFSET_CURRENT;
		}

		// update actual Current
		Capacity.ActualCurrent = Current;
		// update actual Power
		if(Current < 255)	Capacity.ActualPower = (UBat * Current) / 100; // in W higher resolution
		else				Capacity.ActualPower = (UBat * (Current/4)) / 25; // in W

		// update used capacity
		SubCounter += Current;

		// 100mA * 1ms * CAPACITY_UPDATE_INTERVAL = 1 mA * 100 ms * CAPACITY_UPDATE_INTERVAL
		// = 1mA * 0.1s * CAPACITY_UPDATE_INTERVAL = 1mA * 1min / (600 / CAPACITY_UPDATE_INTERVAL)
		// = 1mAh / (36000 / CAPACITY_UPDATE_INTERVAL)
		#define SUB_COUNTER_LIMIT (36000 / CAPACITY_UPDATE_INTERVAL)
		while(SubCounter > SUB_COUNTER_LIMIT)
		{
			Capacity.UsedCapacity++;			// we have one mAh more
			SubCounter -= SUB_COUNTER_LIMIT;	// keep the remaining sub part
		}
	} // EOF check delay update timer
}
