/*
 * gnss.h
 *
 *  Created on: Nov 4, 2024
 *      Author: Raph
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include <stdlib.h>
#include <stdio.h>
#include "string.h"
#include "stm32h7xx_hal.h"

typedef enum {INVALID, NONE, GPS, GLONASS, GALILEO, BEIDOU} Constellation;

typedef struct GNSSData {
	Constellation constellation;
	double_t latitude;
	double_t longitude;
	uint8_t satellites;
	uint32_t GPStime;
	uint32_t date;
	char quality[32];

	uint8_t diffTime;
	float_t diffID;

	char ID[8];

	float_t PDOP;
	float_t HDOP;
	float_t VDOP;

	float_t altitude;
	float_t geoidSep;

	float_t angle;
	float_t speed;
	float_t magnetic;
} GNSSData;

GNSSData* createGNSSData();
uint16_t numberOfTokens(char* data, uint16_t length);
uint8_t convertToDegree(const char* data, const char direction, double_t* dest);

char* getQuality(const char data);

void scanI2C(I2C_HandleTypeDef* i2c);
uint8_t obtainI2CData(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t *buf, uint16_t size);
uint8_t obtainUARTData(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t size);

GNSSData* parseNMEAData(char* data);
// Does NMEA checksum to make sure data is correctly received
uint8_t nmeaChecksum(const char *data, const uint16_t length);
// Verifies data separated by NMEA messages and removes invalid or incorrect data
Constellation verifyData(const char* data, const uint16_t length);
// Check if the data is valid and returns the constellation type
Constellation verifyValidData(const char *data);

#endif /* INC_GNSS_H_ */
