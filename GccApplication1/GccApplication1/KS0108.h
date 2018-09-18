#ifndef KS0108_H_INCLUDED
#define KS0108_H_INCLUDED
/*
||
||  Filename:	 		KS0108.h
||  Title: 			    KS0108 Driver
||  Author: 			Efthymios Koktsidis
||	Email:				efthymios.ks@gmail.com
||  Compiler:		 	AVR-GCC
||	Description:
||	This library can drive KS0108 based GLCD.
||
*/

//----- Headers ------------//
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "IO_Macros.h"
#include "KS0108_Settings.h"
//--------------------------//

//----- Auxiliary data ---------------------------//
#define __GLCD_Pulse_En					1

#define __GLCD_Command_On				0x3F
#define __GLCD_Command_Off				0x3E
#define __GLCD_Command_Set_Address		0x40
#define __GLCD_Command_Set_Page	   		0xB8
#define __GLCD_Command_Display_Start	0xC0

#define __GLCD_Screen_Width          	128
#define __GLCD_Screen_Height         	64
#define	__GLCD_Screen_Line_Height		8
#define __GLCD_Screen_Lines				__GLCD_Screen_Height / __GLCD_Screen_Line_Height
#define __GLCD_Screen_Chips				2

#define __GLCD_BUSY_FLAG				7

enum Chip_t
{
	Chip_1,
	Chip_2,
	Chip_All
};
enum ReadMode_t
{
	GLCD_Increment,
	GLCD_No_Increment
};
enum OperatingMode_t
{
	GLCD_Inverted,
	GLCD_Non_Inverted
};
enum PrintMode_t
{
	GLCD_Overwrite,
	GLCD_Merge
};
enum Color_t
{
	b= 0x00,
	w= 0xFF
};

typedef struct
{
	uint8_t *Name;
	uint8_t Width;
	uint8_t Height;
	uint8_t Lines;
	enum PrintMode_t Mode;
}Font_t;

typedef struct
{
	uint8_t X;
	uint8_t Y;
	enum OperatingMode_t Mode;
	Font_t Font;
}GLCD_t;
//------------------------------------------------//

//----- Prototypes ------------------------------------------------------------//

//-----------------------------------------------------------------------------//
#endif