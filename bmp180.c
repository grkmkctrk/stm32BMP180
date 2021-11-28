/*
 * bmp180.c
 *
 *  Created on: Nov 28, 2021
 *      Author: grkm
 */

#include "bmp180.h"

// calibration datas
short 		    AC1 = 0;
short 		    AC2 = 0;
short 			AC3 = 0;
unsigned short 	AC4 = 0;
unsigned short 	AC5 = 0;
unsigned short 	AC6 = 0;
short 			B1 = 0;
short 			B2 = 0;
short 			MB = 0;
short 			MC = 0;
short 			MD = 0;

// calibration calculation
long 			UT = 0;
long 			UP = 0;
long 			X1 = 0;
long 			X2 = 0;
long 			X3 = 0;
long 			B3 = 0;
long 			B5 = 0;
unsigned long 	B4 = 0;
long 			B6 = 0;
unsigned long 	B7 = 0;

// Altitude Calibration
#define Po 101325

//True Data
float T;
float P;

// start with
void initialBMP180(void){
	readCalibrationBMP180();
}

// read temperature
float readTrueTemp(void){
	calculateTemp();
	return T/10.0;
}

// read press
float readTruePress(uint8_t oss){
	calculatePress(oss);
	return P;
}

// read altitude
float readTrueAltitude(uint8_t oss){
	readTruePress(oss);
	return 44330*(1-(pow((P/(float)Po), 1/5.255)));
}

uint16_t readTempBPM180(void){
	uint8_t data = 0x2E;
	uint8_t tempRAW[2] = {0};

	writeAndRead(
			tempRAW,
			data,
			'T',
			4 // it should be higher then 3
			);

	return ((tempRAW[0] << 8) | tempRAW[1]);
}
uint16_t readPressureBPM180(uint8_t oss){
	uint8_t data = 0x34 + (oss << 6);
	uint8_t pressRaw[3];

	writeAndRead(
			pressRaw,
				data,
				'P',
				oss
				);

	return (((pressRaw[0] << 16) + (pressRaw[1] << 8) + pressRaw[2]) >> (8-oss));
}
void fillData(uint8_t* calibDatas){
	AC1 = (( calibDatas[0] << 8) | calibDatas[1]);
	AC2 = (( calibDatas[2] << 8) | calibDatas[3]);
	AC3 = (( calibDatas[4] << 8) | calibDatas[5]);
	AC4 = (( calibDatas[6] << 8) | calibDatas[7]);
	AC5 = (( calibDatas[8] << 8) | calibDatas[9]);
	AC6 = ((calibDatas[10] << 8) | calibDatas[11]);
	B1 =  ((calibDatas[12] << 8) | calibDatas[13]);
	B2 =  ((calibDatas[14] << 8) | calibDatas[15]);
	MB =  ((calibDatas[16] << 8) | calibDatas[17]);
	MC =  ((calibDatas[18] << 8) | calibDatas[19]);
	MD =  ((calibDatas[20] << 8) | calibDatas[21]);
}
void writeAndRead(uint8_t* raw, uint8_t data, char sens, uint8_t oss){
	uint8_t size;
	HAL_I2C_Mem_Write(
				bmpI2C,
				bmp180Addr,
				BMP180CalibrationReg,
				1,
				&data,
				1,
				1000
				);
	if(sens == 'P'){
		size = 3;
		if(oss == 0) HAL_Delay(5);
		if(oss == 1) HAL_Delay(8);
		if(oss == 2) HAL_Delay(14);
		if(oss == 3) HAL_Delay(26);
	}
	else{
		size = 2;
		HAL_Delay(5);
	}

		HAL_I2C_Mem_Read(
				bmpI2C,
				bmp180Addr,
				BMP180RAWreg,
				1,
				raw,
				size,
				1000
				);
}
void calculateTemp(void){
	UT = readTempBPM180();
	X1 = (UT - AC6)*AC5/(pow(2, 15));
	X2 = (MC*pow(2, 11))/(X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8)/pow(2,4);
}
void calculatePress(uint8_t oss){
	UP = readPressureBPM180(oss);
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>oss);
	P = (B7<0x80000000) ? (B7*2)/B4 : (B7/B4)*2;
	X1 = (P/(pow(2,8)))*(P/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*P)/(pow(2,16));
	P = P + (X1+X2+3791)/(pow(2,4));
}
void readCalibrationBMP180(void){
	uint8_t calibDatas[22] = {0};
	HAL_I2C_Mem_Read(
			bmpI2C,
			bmp180Addr,
			calibDataStart,
			1,
			calibDatas,
			22,
			1000 // If an error occur, change this with HAL_MAX_DELAY
			);

	fillData(calibDatas);
}
