/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include    "mbed.h"
#include    <cstdint>
#include    <stdlib.h>
#include    <string.h>
//#include <sys/types.h>
#include    "platform.h"

SPI         Spi(PA_7, PA_6, PB_3);
DigitalOut  CS0(PB_12);
DigitalOut  CS1(PB_6); 
DigitalIn   CS2(PC_7);      // Interrupted: Measurement completed

void init_IO()
{
    Spi.format(8, 0);
    Spi.frequency(100000);
    CS0 = 1;
    CS1 = 1;
}

volatile uint16_t BckDev = 0xFFFF;

uint16_t Ser_IT()
{
static uint16_t Intr;

    if(CS2 == 0) return(0);
    CS0 = 0;
    Intr  = Spi.write(BckDev) << 8;
    Intr |= Spi.write(0x00);
    CS0 = 1;
    // Debug print
    printf("Ser_IT: SPI returned 0x%04X\n", (unsigned)Intr);
    return(Intr);
}

void Sel_Dev(unsigned short Dev)
{
    char    rD;

    if(Dev != BckDev) {
        // Debug print
        printf("Sel_Dev: change from 0x%04X to 0x%04X (index=%u)\n", (unsigned)BckDev, (unsigned)Dev, (unsigned)Dev);
        CS0 = 0;
        rD = Spi.write((uint8_t)Dev);
        rD = Spi.write(0x00);
        CS0 = 1;
        BckDev = Dev;
    }
}

void Platform_ForceSel(uint16_t val)
{
    char rD;
    CS0 = 0;
    rD = Spi.write((uint8_t)(val & 0xFF));
    rD = Spi.write(0x00);
    CS0 = 1;
    BckDev = val;
    printf("Platform_ForceSel: forced select 0x%04X\n", (unsigned)val);
}

void Platform_SetSpi(uint8_t mode, uint32_t freq)
{
    // map mode to polarity/phase settings: mode 0->(0,0), 1->(1,0), 2->(0,1), 3->(1,1)
    uint8_t pol = (mode == 1 || mode == 3) ? 1 : 0;
    uint8_t pha = (mode == 2 || mode == 3) ? 1 : 0;
    Spi.format(8, (pol<<1)|pha);
    Spi.frequency(freq);
    printf("Platform_SetSpi: mode=%u freq=%u\n", (unsigned)mode, (unsigned)freq);
}

uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
    unsigned char    rD[3];  
	uint8_t status = 255;
	/* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD[0] = Spi.write(RegisterAdress >> 8);
    rD[1] = Spi.write(RegisterAdress & 0x00FF);
    rD[2] = Spi.write(0x00);
    CS1 = 1;
    *p_value = rD[2];
    // Debug
    printf("VL53L8CX_RdByte: addr=0x%04X reg=0x%04X -> 0x%02X\n", (unsigned)p_platform->address, (unsigned)RegisterAdress, (unsigned)rD[2]);
    status = 0;
	return status;
}

uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
    unsigned char    rD[3];    
	uint8_t         status = 255;
	/* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD[0] = Spi.write((RegisterAdress >> 8) | 0x80);
    rD[1] = Spi.write(RegisterAdress & 0x00FF);
    rD[2] = Spi.write(value);
    CS1 = 1;
    // Debug
    printf("VL53L8CX_WrByte: addr=0x%04X reg=0x%04X <- 0x%02X\n", (unsigned)p_platform->address, (unsigned)RegisterAdress, (unsigned)value);
    status = 0;
	return status;
}

uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
    int             n;
    unsigned char   rD;
	uint8_t status = 255;
	
	/* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD = Spi.write((RegisterAdress >> 8) | 0x80);
    rD = Spi.write(RegisterAdress & 0x00FF);
    for(n = 0; n < size; n++) {
        rD = Spi.write(p_values[n]);
    }
    CS1 = 1;
    status = 0;
	return status;
}

uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
    int             n;
    unsigned char   rD;
	uint8_t status = 255;
	
	/* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD = Spi.write(RegisterAdress >> 8);
    rD = Spi.write(RegisterAdress & 0x00FF);
    for(n = 0; n < size; n++) {
        rD = Spi.write(0x00);
        *(p_values + n) = rD;
    }
    CS1 = 1;
    status = 0;
	return status;
}

uint8_t Platform_RdByteDirect(uint16_t RegisterAdress, uint8_t *p_value)
{
    unsigned char rD[3];
    CS1 = 0;
    rD[0] = Spi.write(RegisterAdress >> 8);
    rD[1] = Spi.write(RegisterAdress & 0x00FF);
    rD[2] = Spi.write(0x00);
    CS1 = 1;
    *p_value = rD[2];
    printf("Platform_RdByteDirect: reg=0x%04X -> 0x%02X\n", (unsigned)RegisterAdress, (unsigned)rD[2]);
    return 0;
}

uint8_t Platform_WrByteDirect(uint16_t RegisterAdress, uint8_t value)
{
    unsigned char rD;
    CS1 = 0;
    rD = Spi.write((RegisterAdress >> 8) | 0x80);
    rD = Spi.write(RegisterAdress & 0x00FF);
    rD = Spi.write(value);
    CS1 = 1;
    printf("Platform_WrByteDirect: reg=0x%04X <- 0x%02X\n", (unsigned)RegisterAdress, (unsigned)value);
    return 0;
}

uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	/* Set pin CORE_1V8 to LOW */
	VL53L8CX_WaitMs(p_platform, 100);

	/* Set pin LPN to HIGH */
	/* Set pin AVDD to HIGH */
	/* Set pin VDDIO to HIGH */
	/* Set pin CORE_1V8 to HIGH */
	VL53L8CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
    ThisThread::sleep_for(TimeMs);
    status = 0;
	return status;
}
