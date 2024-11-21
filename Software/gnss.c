#include "gnss.h"

/* Helper functions */
GNSSData* createGNSSData() {
	GNSSData *data = (GNSSData*) malloc(sizeof(GNSSData));
	data->constellation = INVALID;
	data->latitude = 0.0;
	data->longitude = 0.0;
	data->satellites = 0;
	data->GPStime = 0;
	data->date = 0;
	data->quality[0] = '\0';
	data->diffTime = 0;
	data->diffID = 0;
	data->ID[0] = '\0';
	data->PDOP = 0;
	data->HDOP = 0;
	data->VDOP = 0;
	data->altitude = 0.0;
	data->geoidSep = 0.0;
	data->angle = 0.0;
	data->speed = 0.0;
	data->magnetic = 0.0;
	return data;
}

uint16_t numberOfTokens(char *data, uint16_t length) {
	uint16_t i;
	uint16_t t = 0;
	for (i = 0; i < length; i++) {
		if (data[i] == ',') {
			t++;
		}
	}
	return t + 1;
}

uint8_t convertToDegree(const char *data, const char direction, double_t *dest) {
	char *error_ptr;
	uint8_t whole = (uint8_t) (strtol(data, &error_ptr, 10) / 100); //Safely convert to whole number, atoi is unsafe
	if (*error_ptr == '.') {
		double_t decimal = strtod(data, &error_ptr);
		if (*error_ptr == '\0') {
			// Success
			decimal = (double_t) (((double_t) (decimal
					- (double_t) (whole * 100))) / 60.0) + (double_t) (whole);
			if (direction == 'S' || direction == 'W') {
				decimal *= -1;
			}
			*dest = decimal;
			return 0;
		}
	}
	printf("Could not convert to degree: %s%c\r\n", data, direction);
	return -1;
}

char* getQuality(const char data) {
	if (data == '0') {
		return "No fix";
	} else if (data == '1') {
		return "Autonomous";
	} else if (data == '2') {
		return "Differential";
	} else if (data == '4') {
		return "RTK fixed";
	} else if (data == '5') {
		return "RTK Float";
	} else if (data == '6') {
		return "Estimated";
	}
	if (data == 'N') {
		return "No fix";
	} else if (data == 'A') {
		return "Autonomous";
	} else if (data == 'D') {
		return "Differential";
	} else if (data == 'R') {
		return "RTK fixed";
	} else if (data == 'F') {
		return "RTK Float";
	} else if (data == 'E') {
		return "Estimated";
	}
	return '\0';
}

/* Reading data */

void scanI2C(I2C_HandleTypeDef *i2c) {
	uint8_t i = 0;

	/*-[ I2C Bus Scanning ]-*/
	for (i = 1; i < 128; i++) {
		if (HAL_I2C_IsDeviceReady(i2c, (uint16_t) (i << 1), 3, 5) == HAL_OK) {
			printf("0x%X\r\n", i);
		}
	}
	/*--[ Scanning Done ]--*/
}

uint8_t obtainI2CData(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t *buf,
		uint16_t size) {
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(i2c,
			(uint16_t) (addr << 1), 3, 5);
	uint8_t sizeU = 0;
	uint8_t sizeL = 0;
	uint16_t dataSize = 0;
	if (status == HAL_OK) {
		HAL_I2C_Mem_Read(i2c, (uint16_t) (addr << 1), 0xFD, 1, &sizeU,
				sizeof(sizeU), 5);
		HAL_I2C_Mem_Read(i2c, (uint16_t) (addr << 1), 0xFE, 1, &sizeL,
				sizeof(sizeL), 5);
		dataSize = (sizeU << 8) + sizeL;
		if (dataSize > 0) {
			/*printf("Data entering of size %d obtained from %d %d\r\n", dataSize,
			 sizeU, sizeL);*/
			//HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr<<1), 0xFF, 1, pData, size, 1000);
			if (dataSize > size) {
				printf("Too much data: %d, buffer too small\r\n", dataSize);
				dataSize = size;
			}
			HAL_I2C_Mem_Read(i2c, (uint16_t) (addr << 1), 0xFF, 1, buf,
					dataSize, 1000);
			return 0;
		}
	} else {
		printf("Bad status: %d\r\n", status);
	}
	return -1;
}

uint8_t obtainUARTData(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t size) {
	uint8_t sizeU = 0;
	uint8_t sizeL = 0;
	uint16_t dataSize = 0;

}

GNSSData* parseNMEAData(char *data) {
	const char outer_delimiter[] = "\r\n";
	const char inner_delimiter[] = ",";
	char *outer_saveptr = NULL;

	GNSSData *gnssData = createGNSSData();
	uint8_t status = -1;

	char *line = strtok_r(data, outer_delimiter, &outer_saveptr);
	char *lineStart = NULL;

	// Temporary variables
	char *error_ptr;
	float_t tempFloat;
	uint32_t temp32;
	uint8_t temp8;
	uint16_t length;
	uint16_t tokenNum;
	uint16_t i = 0;

	while (line != NULL) {
		lineStart = strchr(line, '$');
		if (lineStart != NULL) { // Realign $ to be start of message
			line = lineStart;
		}
		length = strlen(line); // Length of string
		tokenNum = numberOfTokens(line, length); // Number of tokens in line

		Constellation constellation = verifyData(line, length); // Check headers to make sure they start in NMEA format

		if (constellation != INVALID && tokenNum > 1) { // If data is valid and contains data (more than just the head

			//printf("%s\r\n", line);

			// Tokenise data
			char *lineArray[tokenNum];
			i = 0;
			lineArray[i] = strsep(&line, inner_delimiter);
			while (lineArray[i] != NULL) {
				i++;
				lineArray[i] = strsep(&line, inner_delimiter);
			}

			// Assign data
			gnssData->constellation = constellation;
			if (strncmp("GSA", lineArray[0] + 3, 3) == 0) {
				if (lineArray[2][0] != '1') {
					if (lineArray[1][0] == 'M') {
						strcpy(gnssData->quality, "Manual");
					} else if (lineArray[1][0] == 'A') {
						strcpy(gnssData->quality, "Auto");
					} else {
						strcpy(gnssData->quality, "Unknown");
					}
					tempFloat = strtof(lineArray[tokenNum - 4], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->PDOP = tempFloat;
					}
					tempFloat = strtof(lineArray[tokenNum - 3], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->HDOP = tempFloat;
					}
					tempFloat = strtof(lineArray[tokenNum - 2], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->VDOP = tempFloat;
					}
					if (lineArray[tokenNum - 1][0] == '1') {
						strcpy(gnssData->ID, "GPS");
					} else if (lineArray[tokenNum - 1][0] == '2') {
						strcpy(gnssData->ID, "GLONASS");
					} else if (lineArray[tokenNum - 1][0] == '3') {
						strcpy(gnssData->ID, "GALILEO");
					} else if (lineArray[tokenNum - 1][0] == '5') {
						strcpy(gnssData->ID, "BEIDOU");
					}
					status = 0;
				}
			} else if (strncmp("GSV", lineArray[0] + 3, 3) == 0) {
				printf("GSV: ");
				for (i = 0; i < tokenNum - 1; i++) {
					printf("%s,", lineArray[i]);
				}
				printf("%s\r\n", lineArray[i + 1]);
			} else if (strncmp("VTG", lineArray[0] + 3, 3) == 0) {
				if (lineArray[2][0] == 'T' && lineArray[4][0] == 'M'
						&& lineArray[6][0] == 'N' && lineArray[8][0] == 'K'
						&& lineArray[9][0] != '1') {
					tempFloat = strtof(lineArray[1], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->angle = tempFloat;
					}
					tempFloat = strtof(lineArray[3], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->magnetic = tempFloat;
					}
					tempFloat = strtof(lineArray[5], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->speed = tempFloat;
					}
					char *quality = getQuality(lineArray[9][0]);
					if (quality[0] != '\0') {
						strcpy(gnssData->quality, quality);
					}
					status = 0;
				}
			} else if (strncmp("GGA", lineArray[0] + 3, 3) == 0) {
				if (lineArray[10][0] == 'M' && lineArray[12][0] == 'M'
						&& lineArray[6][0] != '0') {
					temp32 = strtol(lineArray[1], &error_ptr, 10);
					if (*error_ptr == '.') {
						gnssData->GPStime = temp32;
						status = 0;
					}
					convertToDegree(lineArray[2], lineArray[3][0],
							&(gnssData->latitude));
					convertToDegree(lineArray[4], lineArray[5][0],
							&(gnssData->longitude));
					char *quality = getQuality(lineArray[6][0]);
					if (quality[0] != '\0') {
						strcpy(gnssData->quality, quality);
					}
					temp8 = strtol(lineArray[7], &error_ptr, 10);
					if (*error_ptr == '\0') {
						gnssData->satellites = temp8;
					}
					tempFloat = strtof(lineArray[8], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->HDOP = tempFloat;
					}
					tempFloat = strtof(lineArray[9], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->altitude = tempFloat;
					}
					tempFloat = strtof(lineArray[11], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->geoidSep = tempFloat;
					}
					temp8 = strtol(lineArray[13], &error_ptr, 10);
					if (*error_ptr == '\0') {
						gnssData->diffTime = temp8;
					}
					tempFloat = strtol(lineArray[14], &error_ptr, 10);
					if (*error_ptr == '*') {
						gnssData->diffID = temp8;
					}
					status = 0;
				}
			} else if (strncmp("RMC", lineArray[0] + 3, 3) == 0) {
				if (lineArray[2][0] == 'A' && lineArray[12][0] != 'N') {
					temp32 = strtol(lineArray[1], &error_ptr, 10);
					if (*error_ptr == '.') {
						gnssData->GPStime = temp32;
						status = 0;
					}
					convertToDegree(lineArray[3], lineArray[4][0],
							&(gnssData->latitude));
					convertToDegree(lineArray[5], lineArray[6][0],
							&(gnssData->longitude));
					tempFloat = strtof(lineArray[7], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->speed = tempFloat;
					}
					tempFloat = strtof(lineArray[8], &error_ptr);
					if (*error_ptr == '\0') {
						gnssData->angle = tempFloat;
					}
					temp32 = strtol(lineArray[9], &error_ptr, 10);
					if (*error_ptr == '\0') {
						gnssData->date = temp32;
					}
					tempFloat = strtof(lineArray[10], &error_ptr);
					if (*error_ptr == '\0') {
						if (lineArray[11][0] == 'W') {
							tempFloat *= -1;
						}
						gnssData->magnetic = tempFloat;
					}
					char *quality = getQuality(lineArray[12][0]);
					if (quality[0] != '\0') {
						strcpy(gnssData->quality, quality);
					}
					status = 0;
				} else if (lineArray[1][0] != '\0') { //If time still available
					temp32 = strtol(lineArray[1], &error_ptr, 10);
					if (*error_ptr == '.') {
						gnssData->GPStime = temp32;
						status = 0;
					}
				}
			} else if (strncmp("GLL", lineArray[0] + 3, 3) == 0) {
				if (lineArray[6][0] == 'A') {
					convertToDegree(lineArray[1], lineArray[2][0],
							&(gnssData->latitude));
					convertToDegree(lineArray[3], lineArray[4][0],
							&(gnssData->longitude));
					temp32 = strtol(lineArray[5], &error_ptr, 10);
					if (*error_ptr == '.') {
						gnssData->GPStime = temp32;
					}
					status = 0;
				} else if (lineArray[5][0] != '\0') { //If time still available
					temp32 = strtol(lineArray[5], &error_ptr, 10);
					if (*error_ptr == '.') {
						gnssData->GPStime = temp32;
						status = 0;
					}
				}
			} else {
				// Reformat into original and print
				printf("Cannot parse line: ");
				for (i = 0; i < tokenNum - 1; i++) {
					printf("%s,", lineArray[i]);
				}
				printf("%s\r\n", lineArray[i + 1]);
			}

		}
		//HAL_Delay(100);
		//printf("sdasdasdc\n");
		//HAL_Delay(100);
//		printf("c\n");

		//	printf("d\n");
		line = strtok_r(NULL, outer_delimiter, &outer_saveptr);
		//printf("Next line: %s\r\n", line);
	}

	if (status) {
		printf("No valid data\n");
		free(gnssData);
		gnssData = NULL;
	}
	return gnssData;
}

uint8_t nmeaChecksum(const char *data, const uint16_t length) {
	uint16_t i;
	uint8_t check = 0;
	if ((data + length - 3)[0] == '*') {
		for (i = 1; i < length - 3; i++) {
			check ^= (uint8_t) data[i];
		}
		if (check == (uint8_t) strtol(data + length - 2, NULL, 16)) {
			return 0;
		}
	}
	return -1;
}

Constellation verifyData(const char *data, const uint16_t length) {
	if (length > 9) { //6 for the $GXXXX and 3 for *XX
		if (nmeaChecksum(data, length) == 0) {
			if (strncmp("$GNTXT", data, 6) == 0) {
				printf("%s\r\n", data); //Print out any GNTXT data
				return INVALID;
			} else {
				return verifyValidData(data);
			}
		} else {
			printf("Failed NMEA - ");
		}
	}
	printf("Invalid data: %s\r\n", data);
	return INVALID;
}

Constellation verifyValidData(const char *data) {
	Constellation constellation = INVALID;
	/*char* head[4] = { 0 };
	 strncpy(head, data + 3, 3);
	 printf("%s\r\n", head);
	 if (strncmp("GSA", head, 3) == 0) {
	 printf("GNGSA!!!!\r\n");
	 } else if (strncmp("GSV", head, 3) == 0) {
	 correct = 1;
	 } else if (strncmp("VTG", head, 3) == 0) {
	 printf("GNVTG!!!!\r\n");
	 } else if (strncmp("GGA", head, 3) == 0) {
	 printf("GNGGA!!!!\r\n");
	 } else if (strncmp("RMC", head, 3) == 0) {
	 printf("GNRMC!!!!\r\n");
	 } else if (strncmp("GLL", head, 3) == 0) {
	 printf("GNGGL!!!!\r\n");
	 }*/
	if (strncmp("$GP", data, 3) == 0) {
		constellation = GPS;
	} else if (strncmp("$GL", data, 3) == 0) {
		constellation = GLONASS;
	} else if (strncmp("$GA", data, 3) == 0) {
		constellation = GALILEO;
	} else if (strncmp("$GB", data, 3) == 0 || strncmp("$BD", data, 3) == 0) {
		constellation = BEIDOU;
	} else if (strncmp("$GN", data, 3) == 0) {
		constellation = NONE;
	}
	return constellation;
}
