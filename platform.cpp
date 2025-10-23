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

// I2C implementation for VL53L8CX platform layer
// Uses I2C peripheral for register access. Assumes p_platform->address
// contains the 8-bit I2C address (for example default 0x52 as in API header).

// NOTE: Pins below (PB_7 SDA, PB_6 SCL) are typical for NUCLEO-F446RE I2C1.
// Adjust if your wiring differs.
I2C         i2c(PB_7, PB_6);
DigitalIn   IRQ_PIN(PC_7); // Measurement complete / IRQ pin (was CS2)

void init_IO()
{
    // Initialize I2C bus (400 kHz)
    i2c.frequency(400000);
}

// For compatibility with existing API we keep these symbols.
volatile uint16_t BckDev = 0xFFFF; // not used for I2C

uint16_t Ser_IT()
{
    // Original SPI-based Ser_IT returned a 16-bit mask read via SPI.
    // On I2C-based wiring there is often an IRQ pin that simply goes low/high.
    // Here we return 0 when IRQ is asserted (low), or 0xFFFF otherwise.
    // Main program may expect bitfields; adapt main if you need specific bits.
    if(IRQ_PIN == 0) return 0xFFFF; // IRQ active
    return 0;
}

void Sel_Dev(unsigned short Dev)
{
    // No device selection required for I2C — selection is by I2C address.
    (void)Dev; // keep signature
}

uint8_t VL53L8CX_RdByte(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value)
{
    char tx[2];
    tx[0] = (char)(RegisterAdress >> 8);
    tx[1] = (char)(RegisterAdress & 0xFF);

    int addr = (int)p_platform->address; // expects 8-bit address (e.g. 0x52)

    // Write register address with repeated start, then read one byte
    if (i2c.write(addr, tx, 2, true) != 0) return VL53L8CX_STATUS_ERROR;

    char rx;
    if (i2c.read(addr, &rx, 1) != 0) return VL53L8CX_STATUS_ERROR;

    *p_value = (uint8_t)rx;
    return VL53L8CX_STATUS_OK;
}

uint8_t VL53L8CX_WrByte(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value)
{
    char tx[3];
    tx[0] = (char)(RegisterAdress >> 8);
    tx[1] = (char)(RegisterAdress & 0xFF);
    tx[2] = (char)value;

    int addr = (int)p_platform->address;
    if (i2c.write(addr, tx, 3) != 0) return VL53L8CX_STATUS_ERROR;
    return VL53L8CX_STATUS_OK;
}

uint8_t VL53L8CX_WrMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    // Build buffer: 2 bytes register address + data
    uint32_t total = 2 + size;
    char *tx = (char *)malloc(total);
    if(!tx) return VL53L8CX_STATUS_ERROR;
    tx[0] = (char)(RegisterAdress >> 8);
    tx[1] = (char)(RegisterAdress & 0xFF);
    for(uint32_t i=0;i<size;i++) tx[2+i] = (char)p_values[i];

    int addr = (int)p_platform->address;
    int ret = i2c.write(addr, tx, total);
    free(tx);
    return (ret == 0) ? VL53L8CX_STATUS_OK : VL53L8CX_STATUS_ERROR;
}

uint8_t VL53L8CX_RdMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    char tx[2];
    tx[0] = (char)(RegisterAdress >> 8);
    tx[1] = (char)(RegisterAdress & 0xFF);
    int addr = (int)p_platform->address;

    if (i2c.write(addr, tx, 2, true) != 0) return VL53L8CX_STATUS_ERROR;
    if (i2c.read(addr, (char*)p_values, size) != 0) return VL53L8CX_STATUS_ERROR;

    return VL53L8CX_STATUS_OK;
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
