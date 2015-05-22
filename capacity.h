#ifndef _CAPACITY_H
#define _CAPACITY_H

typedef struct
{
	unsigned short ActualCurrent; // in 0.1A Steps
	unsigned short ActualPower;   // in 0.1W
	unsigned short UsedCapacity;  // in mAh
	unsigned char MinOfMaxPWM;	  // BL Power Limit
} __attribute__((packed)) Capacity_t;

extern Capacity_t Capacity;

void Capacity_Init(void);
void Capacity_Update(void);

#endif //_CAPACITY_H

