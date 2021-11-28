/*
 * bmp180.h
 *
 *  Created on: Nov 28, 2021
 *      Author: grkm
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define bmpI2C &hi2c1
#define bmp180Addr 0xEE

#define calibDataStart 0xAA
#define BMP180CalibrationReg 0xF4
#define BMP180RAWreg 0xF6

void readCalibrationBMP180(void);
void fillData(uint8_t*);
void calculateTemp(void);
void calculatePress(uint8_t oss);
void writeAndRead(
		uint8_t* raw,
		uint8_t data,
		char sens,
		uint8_t oss
		);

// RAW
uint16_t readTempBPM180(void);
uint16_t readPressureBPM180(uint8_t oss);
// TRUE
float readTrueTemp(void);
float readTruePress(uint8_t oss);
float readTrueAltitude(uint8_t oss);

void initialBMP180(void);

#endif /* INC_BMP180_H_ */
