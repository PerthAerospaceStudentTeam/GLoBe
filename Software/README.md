# Software usage
These drivers are written in C for the STM32H7, this is the code in the main loop
Make sure to `#include "gnss.h"` and initialise the pins

### Usage for STM32's UART protocol
```c
uint16_t size = 2000;
uint8_t *pData = malloc(sizeof(uint8_t) * size);
HAL_Delay(1000);
uint32_t Timer = HAL_GetTick();
uint16_t i = 0;

while (1) {
    // IF USING UART
    while (HAL_UART_Receive(&huart1, pData + i, 1, 100) == HAL_OK && i < size - 1) {
        i++;
    }
    *(pData + i) = (uint8_t) '\0';
    if ((HAL_GetTick() - Timer) >= 1000) {
        i = 0;

        GNSSData *gnssData = parseNMEAData((char*) pData);
        if (gnssData != NULL) {
            printf("Data:%lf,%lf,%ld,%d,%f,%ld,%f,%f,%f,%f,%f,%f\r\n",
                    gnssData->latitude, gnssData->longitude,
                    gnssData->GPStime, gnssData->satellites,
                    gnssData->altitude, gnssData->date, gnssData->angle,
                    gnssData->speed, gnssData->PDOP, gnssData->HDOP,
                    gnssData->VDOP, gnssData->geoidSep);
        }
        free(gnssData);
        gnssData = NULL;
        printf("%s", pData);

        Timer = HAL_GetTick();
    }

}
free(pData);
pData = NULL;
```

### Usage for STM32's I2C protocol
```c
uint8_t addr = 0x42;
uint16_t size = 2000;
uint8_t *pData = malloc(sizeof(uint8_t) * size);
HAL_Delay(1000);
scanI2C(&hi2c1);
uint32_t Timer = HAL_GetTick();

while (1) {
    // IF USING I2C
    if ((HAL_GetTick() - Timer) > 500) {
        if (!obtainI2CData(&hi2c1, addr, pData, size)) {
            GNSSData *gnssData = parseNMEAData((char*) pData);
            if (gnssData != NULL) {
                printf("Data:%lf,%lf,%d,%d,%f,%d,%f,%f,%f,%f,%f,%f\r\n", gnssData->latitude,
                gnssData->longitude, gnssData->GPStime,
                gnssData->satellites, gnssData->altitude,
                gnssData->date, gnssData->angle, gnssData->speed,
                gnssData->PDOP, gnssData->HDOP, gnssData->VDOP,
                gnssData->geoidSep);
            }
            free(gnssData);
            gnssData = NULL;
        } else {
            printf("No data\r\n");
        }
        Timer = HAL_GetTick();
    }
}
free(pData);
pData = NULL;
```