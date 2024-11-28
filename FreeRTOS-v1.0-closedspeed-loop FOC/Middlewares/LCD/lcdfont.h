#ifndef __LCDFONT_H
#define __LCDFONT_H 	   

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"
extern const unsigned char ascii_1206[][12];

extern const unsigned char ascii_1608[][16];
extern const unsigned char ascii_2412[][48];
extern const unsigned char ascii_3216[][64];

typedef struct 
{
	unsigned char Index[2];	
	unsigned char Msk[24];
}typFNT_GB12; 

extern const typFNT_GB12 tfont12[300];

typedef struct 
{
	unsigned char Index[2];	
	unsigned char Msk[32];
}typFNT_GB16; 

extern const typFNT_GB16 tfont16[300];

typedef struct 
{
	unsigned char Index[2];	
	unsigned char Msk[72];
}typFNT_GB24; 

extern const typFNT_GB24 tfont24[300];

typedef struct 
{
	unsigned char Index[2];	
	unsigned char Msk[128];
}typFNT_GB32; 

extern const typFNT_GB32 tfont32[300];

#ifdef __cplusplus
}
#endif


#endif


