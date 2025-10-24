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
#include    <chrono>
//#include <sys/types.h>
#include    "platform.h"
#include    "vl53l8cx_api.h"

// I2C implementation for VL53L8CX platform layer
// Uses I2C peripheral for register access. Assumes p_platform->address
// contains the 8-bit I2C address (for example default 0x52 as in API header).

// NOTE: Pins below (PB_7 SDA, PB_6 SCL) are typical for NUCLEO-F446RE I2C1.
// Adjust if your wiring differs.
I2C         i2c(PB_7, PB_6);
DigitalIn   IRQ_PIN(PC_7); // Measurement complete / IRQ pin (was CS2)
DigitalOut  LPN_PIN_OUTPUT(PLATFORM_LPN_PIN);

void init_IO()
{
    // Initialize I2C bus (400 kHz)
    i2c.frequency(400000);

    // If PLATFORM_LPN_PIN is defined, drive it high to release sensor from reset
    if (PLATFORM_LPN_PIN != NC) {
        LPN_PIN_OUTPUT = 1; // Release reset (active low)
        ThisThread::sleep_for(std::chrono::milliseconds(10));
    }
}

// For compatibility with existing API we keep these symbols.
volatile uint16_t BckDev = 0xFFFF; // not used for I2C

// Address mode detection for I2C: some code passes 8-bit address (0x52),
// others expect 7-bit (0x29). We detect which one the I2C API expects.
static int i2c_addr_mode = 0; // 0=unknown, 1=use as-is, 2=use >>1 (7-bit)

static int resolved_addr(uint16_t raw)
{
    if(i2c_addr_mode == 1) return (int)raw;
    if(i2c_addr_mode == 2) return (int)(raw >> 1);
    // unknown -> return raw for a first try
    return (int)raw;
}

void print_i2c_scan()
{
    printf("I2C scan start...\n");
    char buf[1] = {0};
    /* Print pin states to help debugging: IRQ line and whether an LPN pin is
       defined (modules often have XSHUT/LPn that must be released for I2C to
       respond). */
    printf("IRQ_PIN state: %d\n", (int)IRQ_PIN.read());
    if (PLATFORM_LPN_PIN != NC) {
        printf("PLATFORM_LPN_PIN defined (PinName=%d)\n", (int)PLATFORM_LPN_PIN);
    } else {
        printf("PLATFORM_LPN_PIN == NC (not defined)\n");
    }

    /* mbed I2C API expects the 8-bit address (7-bit << 1). Probe using 8-bit
       addresses to avoid ambiguity. Addresses 0x02..0xFE (even) correspond
       to 7-bit 0x01..0x7F. */
    for(int a7 = 1; a7 < 128; a7++) {
        int addr8 = a7 << 1; // 8-bit address for mbed I2C
        if(i2c.write(addr8, buf, 0) == 0) {
            printf(" ACK at 7bit:0x%02X 8bit:0x%02X\n", a7, addr8);
        }
    }
    printf("I2C scan end\n");
}

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

    int raw = (int)p_platform->address;
    int addr = resolved_addr(raw);

    // If address mode unknown, try as-is first, then try >>1 (7-bit)
    if(i2c_addr_mode == 0) {
        if (i2c.write(addr, tx, 2, true) == 0) {
            char rx;
            if (i2c.read(addr, &rx, 1) == 0) {
                i2c_addr_mode = 1; // as-is works
                printf("I2C: detected address mode = as-is (8-bit)\n");
                *p_value = (uint8_t)rx;
                return VL53L8CX_STATUS_OK;
            }
        }
        // try 7-bit
        addr = raw >> 1;
        if (i2c.write(addr, tx, 2, true) == 0) {
            char rx;
            if (i2c.read(addr, &rx, 1) == 0) {
                i2c_addr_mode = 2; // 7-bit
                printf("I2C: detected address mode = 7-bit\n");
                *p_value = (uint8_t)rx;
                return VL53L8CX_STATUS_OK;
            }
        }
        return VL53L8CX_STATUS_ERROR;
    }

    // Known mode: use resolved addr
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

    int raw = (int)p_platform->address;
    int addr = resolved_addr(raw);

    if(i2c_addr_mode == 0) {
        if (i2c.write(addr, tx, 3) == 0) {
            i2c_addr_mode = 1; printf("I2C: detected address mode = as-is (8-bit)\n"); return VL53L8CX_STATUS_OK;
        }
        addr = raw >> 1;
        if (i2c.write(addr, tx, 3) == 0) {
            i2c_addr_mode = 2; printf("I2C: detected address mode = 7-bit\n"); return VL53L8CX_STATUS_OK;
        }
        return VL53L8CX_STATUS_ERROR;
    }

    return (i2c.write(addr, tx, 3) == 0) ? VL53L8CX_STATUS_OK : VL53L8CX_STATUS_ERROR;
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

    int raw = (int)p_platform->address;
    int addr = resolved_addr(raw);
    int ret;

    if(i2c_addr_mode == 0) {
        ret = i2c.write(addr, tx, total);
        if(ret == 0) { i2c_addr_mode = 1; printf("I2C: detected address mode = as-is (8-bit)\n"); free(tx); return VL53L8CX_STATUS_OK; }
        addr = raw >> 1;
        ret = i2c.write(addr, tx, total);
        if(ret == 0) { i2c_addr_mode = 2; printf("I2C: detected address mode = 7-bit\n"); free(tx); return VL53L8CX_STATUS_OK; }
        free(tx);
        return VL53L8CX_STATUS_ERROR;
    }

    ret = i2c.write(addr, tx, total);
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
    int raw = (int)p_platform->address;
    int addr = resolved_addr(raw);

    if(i2c_addr_mode == 0) {
        if (i2c.write(addr, tx, 2, true) == 0) {
            if (i2c.read(addr, (char*)p_values, size) == 0) { i2c_addr_mode = 1; printf("I2C: detected address mode = as-is (8-bit)\n"); return VL53L8CX_STATUS_OK; }
        }
        addr = raw >> 1;
        if (i2c.write(addr, tx, 2, true) == 0) {
            if (i2c.read(addr, (char*)p_values, size) == 0) { i2c_addr_mode = 2; printf("I2C: detected address mode = 7-bit\n"); return VL53L8CX_STATUS_OK; }
        }
        return VL53L8CX_STATUS_ERROR;
    }

    if (i2c.write(addr, tx, 2, true) != 0) return VL53L8CX_STATUS_ERROR;
    if (i2c.read(addr, (char*)p_values, size) != 0) return VL53L8CX_STATUS_ERROR;

    return VL53L8CX_STATUS_OK;
}
uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
    /* If PLATFORM_LPN_PIN is defined (not NC), toggle it to reset the sensor.
       Many modules expose XSHUT/LPn; pulsing it low then high performs a reset.
    */
    if (PLATFORM_LPN_PIN != NC) {
        // Drive low
        LPN_PIN_OUTPUT = 0;
        VL53L8CX_WaitMs(p_platform, 10);
        // Keep low for a short reset
        VL53L8CX_WaitMs(p_platform, 100);
        // Release (drive high)
        LPN_PIN_OUTPUT = 1;
        VL53L8CX_WaitMs(p_platform, 100);
    } else {
        /* No LPN pin defined: user must ensure module is powered and out of reset
           before running. */
        VL53L8CX_WaitMs(p_platform, 200);
    }

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
    ThisThread::sleep_for(std::chrono::milliseconds(TimeMs));
    status = 0;
	return status;
}
