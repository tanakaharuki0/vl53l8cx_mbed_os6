//----------------------------------------------------------------
// mbed6 NUCLEO-F446RE
//----------------------------------------------------------------
#include    "mbed.h"
#include <cstdint>
#include    <stdlib.h>
#include    <string.h>
#include    "vl53l8cx_api.h"
#include    "vl53l8cx_buffers.h"
#include    "platform.h"
#include    "rtos/ThisThread.h"

static BufferedSerial serial_vcp(PA_2, PA_3, 115200);
VL53L8CX_Configuration		Dev;

#define SENSOR_COUNT 3

VL53L8CX_ResultsData 	Results;		// Results data from VL53L8CX 

//---------------------------------------------------------------------------
//  Serial_vcp Input
//---------------------------------------------------------------------------
char    ucmd[64];
int     ucmd_p = 0;

int get_Vcp()
{
    int     n, k;
    char    buf[64];

    k = serial_vcp.readable();
    if(k == 0) return(0);
    k = serial_vcp.read(buf, sizeof(buf));
    if(k != 0) {
        for(n = 0; n < k; n++) {
            if(buf[n] == '\r' || buf[n] == '\n') {
                ucmd[ucmd_p] = 0;
                ucmd_p = 0;
                return(k);
            } else {
                ucmd[ucmd_p] = buf[n];
                if(ucmd_p < 64) ucmd_p++;
            }
        }
    }
    return(0);
}

//----------------------------------------------------------------
// Example_1_Ranging_Basic(The VL53L8CX ULD package)
//----------------------------------------------------------------
void Ranging_Basic(uint16_t DevAddr)
{
    uint8_t 				status, loop, isAlive, isReady, i;
	VL53L8CX_Configuration 	Dev;			// Sensor configuration 

    Dev.platform.address = DevAddr;

    // (Optional) Check if there is a VL53L8CX sensor connected
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status) {
		printf("VL53L8CX not detected at requested address\n");
		return;
	}

    // (Mandatory) Init VL53L8CX sensor
	status = vl53l8cx_init(&Dev);
	if(status) {
		printf("VL53L8CX ULD Loading failed\n");
		return;
	}
    printf("VL53L8CX ULD ready ! (Version : %s)\n", VL53L8CX_API_REVISION);

    // Ranging loop
    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 1);
	if(status) {
		printf("vl53l8cx_set_ranging_frequency_hz failed, status %u\n", status);
		return;
	}
    status = vl53l8cx_start_ranging(&Dev);
    loop = 0;
	while(loop < 10) {
        status = vl53l8cx_check_data_ready(&Dev, &isReady);
        if(isReady) {
			vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++) {
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
					Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}
        VL53L8CX_WaitMs(&(Dev.platform), 5);
    }
}

//----------------------------------------------------------------
// Multiple Sensor(by Kizaki)
//----------------------------------------------------------------
VL53L8CX_Configuration 	MDev[SENSOR_COUNT];

int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t     status, isAlive;
    MDev[DevAddr].platform.address = DevAddr;
    printf("Init_Sensor: DevAddr=%u\n", (unsigned)DevAddr);
    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    printf("  is_alive returned status=%u isAlive=%u\n", (unsigned)status, (unsigned)isAlive);
    if(status) {
        printf("VL53L8CX ULD Loading failed_alive[%d]\n", DevAddr);
        return(0);
    }
    status = vl53l8cx_init(&MDev[DevAddr]);
    printf("  init returned status=%u\n", (unsigned)status);
    if(status) {
        printf("VL53L8CX ULD Loading failed_init[%d]\n", DevAddr);
        return(0);
    }
    printf("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
	if(status) {
		printf("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
		return(0);
	}
    //status = vl53l8cx_start_ranging(&MDev[DevAddr]);
    return(1);
}

void Start_Ranging(uint16_t DevAddr)
{
    uint8_t     status;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
}

uint8_t DevAddr[SENSOR_COUNT];
uint8_t ReStart[SENSOR_COUNT] = { 0 };
uint8_t NumRdy[SENSOR_COUNT] = { 0 };

void Gget_Ranging()
{
    uint8_t     status, loop, isAlive, isReady;
    char        i;
    int         k;

    for(k = 0; k < SENSOR_COUNT; k++) DevAddr[k] = 0xFF;
    k = Ser_IT();       // In The platform.cpp
    if(k == 0) return;

    // Map lower 3 bits from Ser_IT() to device indices 0..2
    {
        int mask = k;
        for(char d = 0; d < SENSOR_COUNT; d++) {
            if(mask & (1 << d)) DevAddr[d] = d;
        }
    }

    for(k = 0; k < SENSOR_COUNT; k++) {
        if(DevAddr[k] != 0xFF) {
            MDev[DevAddr[k]].platform.address = DevAddr[k];
            vl53l8cx_get_ranging_data(&MDev[DevAddr[k]], &Results);
            //printf("[%d]Print data no : %3u\n", DevAddr[k], MDev[DevAddr[k]].streamcount);
            printf("[%2d] ", DevAddr[k]);
			for(i = 0; i < 16; i++) {
                if(Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i] == 5) {
                    printf("[%4d]", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
                } else {
                    printf("--%02X--", Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                }
			}
            printf(" R[%3d]", NumRdy[k]);
            printf(" S[%d]", ReStart[k]);
			printf("\n");
            NumRdy[k] = 0;
        } else {
            NumRdy[k]++;
        }
    }
    for(k = 0; k < SENSOR_COUNT; k++) {
        if(NumRdy[k] > 250) {
            printf("Restart DevAddr[%d] NumRdy[%d]\n", k, NumRdy[k]);
            Start_Ranging(k);
            ReStart[k]++;
        }
    }
}

//----------------------------------------------------------------
// Main
//----------------------------------------------------------------
int main()
{
    int     n, m, k, InitError;
    
    ThisThread::sleep_for(500ms);
    init_IO();      // In The platform.cpp
    ThisThread::sleep_for(500ms);
    printf("TOF Sens Test Start\n");

    InitError = 1;
    while(InitError) {
    for(n = 0, InitError = 0; n < SENSOR_COUNT; n++) {
            if(Init_Sensor(n, 1) == 0) InitError = 1;
        }
        ThisThread::sleep_for(500ms);
    }
    printf("Ranging Start\n");
    for(n = 0; n < SENSOR_COUNT; n++) {
        vl53l8cx_start_ranging(&MDev[n]);
    }

    while (true) {
        Gget_Ranging();
        k = get_Vcp();
        if(k != 0) {
            //-------------------------------------------------------------------
            // Sampling rate Setup
            // f frequency[1--60]
            //-------------------------------------------------------------------
            if(ucmd[0] == 'f') {
                for( n = 1; ucmd[n] == ' ' || ucmd[n] == '\t'; n++);
                sscanf(&ucmd[n], "%d", &m);
                if(m >= 1 && m <= 60) {
                    for(n = 0; n < SENSOR_COUNT; n++) {
                        MDev[n].platform.address = n;
                        vl53l8cx_set_ranging_frequency_hz(&MDev[n], m);
                    }
                    printf("Sampling Rate Setup[f=%d]\n", m);
                } else {
                    printf("Error Sampling Rate Setup[f %d]\n", m);
                }
            }
        }
    }
}
