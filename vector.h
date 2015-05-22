#ifndef _VECTOR_H
#define _VECTOR_H

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
} __attribute__((packed)) vector32_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__((packed)) vector16_t;


#endif //_VECTOR_H
