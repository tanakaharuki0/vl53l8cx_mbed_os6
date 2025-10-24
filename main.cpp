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
VL53L8CX_Configuration 	MDev[11];

int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t     status, isAlive;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    if(status) {
		printf("VL53L8CX ULD Loading failed_alive[%d]\n", DevAddr);
		return(0);
	}
    status = vl53l8cx_init(&MDev[DevAddr]);
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

uint8_t DevAddr[11];
uint8_t ReStart[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t NumRdy[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void Gget_Ranging()
{
    uint8_t     status, loop, isAlive, isReady;
    char        i;
    int         k;

    for(k = 0; k < 11; k++) DevAddr[k] = 0xFF;
    k = Ser_IT();       // In The platform.cpp
    if(k == 0) return;

    if((k & 0x0020) != 0) DevAddr[10] = 10;
    if((k & 0x0040) != 0) DevAddr[9] = 9;
    if((k & 0x0080) != 0) DevAddr[8] = 8;
    if((k & 0x0100) != 0) DevAddr[7] = 7;
    if((k & 0x0200) != 0) DevAddr[6] = 6;
    if((k & 0x0400) != 0) DevAddr[5] = 5;
    if((k & 0x0800) != 0) DevAddr[4] = 4;
    if((k & 0x1000) != 0) DevAddr[3] = 3;
    if((k & 0x2000) != 0) DevAddr[2] = 2;
    if((k & 0x4000) != 0) DevAddr[1] = 1;
    if((k & 0x8000) != 0) DevAddr[0] = 0;
    for(k = 0; k < 11; k++) {
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
    for(k = 0; k < 11; k++) {
        if(NumRdy[k] > 250) {
            printf("Restart DevAddr[%d] NumRdy[%d]\n", k, NumRdy[k]);
            Start_Ranging(k);
            ReStart[k]++;
        }
    }
}

//----------------------------------------------------------------
// Main (single I2C sensor)
//----------------------------------------------------------------
int main()
{
    uint8_t status, isAlive, isReady, i;

    ThisThread::sleep_for(500ms);
    init_IO();      // I2C init in platform.cpp
    ThisThread::sleep_for(500ms);
    printf("VL53L8CX Single I2C Ranging Test Start\n");

    // Debug: print I2C scan to find which addresses ACK on the bus
    print_i2c_scan();

    // Use default I2C address defined in API (0x52)
    Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

    // Check sensor presence
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if(status || !isAlive) {
        printf("VL53L8CX not detected at address 0x%02X (status=%u isAlive=%u)\n",
               Dev.platform.address, status, isAlive);
        return 0;
    }

    // Initialize sensor (loads firmware)
    status = vl53l8cx_init(&Dev);
    if(status) {
        printf("vl53l8cx_init failed, status %u\n", status);
        return 0;
    }
    printf("VL53L8CX initialized (API: %s)\n", VL53L8CX_API_REVISION);

    // Set ranging frequency (Hz)
    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 10);
    if(status) printf("set_ranging_frequency_hz failed %u\n", status);

    // Start ranging
    status = vl53l8cx_start_ranging(&Dev);
    if(status) {
        printf("start_ranging failed %u\n", status);
        return 0;
    }

    // Main loop: poll for data and print zones
    while(true) {
        status = vl53l8cx_check_data_ready(&Dev, &isReady);
        if(status != VL53L8CX_STATUS_OK) {
            printf("check_data_ready error %u\n", status);
            VL53L8CX_WaitMs(&(Dev.platform), 10);
            continue;
        }

        if(isReady) {
            status = vl53l8cx_get_ranging_data(&Dev, &Results);
            if(status == VL53L8CX_STATUS_OK) {
                printf("Stream %3u:\n", Dev.streamcount);
                for(i = 0; i < 16; i++) {
                    uint8_t st = Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i];
                    int32_t d = Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i];
                    // Status 5 = valid with reflector, Status 9 = valid no reflector
                    if(st == 5 || st == 9) {
                        printf("Zone %2u: %4d mm\n", i, d);
                    } else {
                        printf("Zone %2u: status=%02u\n", i, st);
                    }
                }
                printf("\n");
            } else {
                printf("get_ranging_data failed %u\n", status);
            }
        }

        VL53L8CX_WaitMs(&(Dev.platform), 50);
    }

    return 0;
}
