/*
 * MLX90614.h
 * Header file for the SparkFun IR Thermometer Library
 *
 * By: Andrew Kent
 *
 * 					     _.-''''''-._
 *				     _.-'			  '-.
 *			       .'				  _  '-
 *			      /				    -' '-_ '_
 *			     /			        \     '8888
 *			    /					 \    888888
 *			   /					  \    8888
 *		      /					       \
 * 		     /						    \
 * 		    /						     \
 * 		   /						      \
 *    ____/__  ______  ______  ______  ____\__
 *   /       \/      \/      \/      \/       \
 *  /										    \
 * |										     |
 *  \										    /
 *   \										   /
 *    \______/\______/\______/\______/\_______/
 *
 * This Library is based on:
 * SparkFunMLX90614.cpp
 * The SparkFun IR Thermometer Library
 *
 * Jim Lindblom @ SparkFun Electronics
 * October 23, 2015
 * https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library
 *
 * This file defines the SMBus hardware interface(s) for the MLX90614 IR thermometer
 * and abstracts temperature measurments and other features of the MLX90614
 *
 * Development environment specifics:
 * SparkFun IR Thermometer Evaluation Board - MLX90614
 */

#ifndef SRC_MLX90614_H_
#define SRC_MLX90614_H_

#include "main.h"
#include "math.h"
#include "float.h"
#include "stdbool.h"

//////////////////////////////////
// MLX90614 Default I2C Address //
//////////////////////////////////
#define MLX90614_DEFAULT_ADDRESS 0x5A

///////////////////////////////////////
// MLX90614 RAM and EEPROM Addresses //
///////////////////////////////////////
#define MLX90614_REGISTER_TA      0x06
#define MLX90614_REGISTER_TOBJ1	  0x07
#define MLX90614_REGISTER_TOBJ2	  0x08
#define MLX90614_REGISTER_TOMAX   0x20
#define MLX90614_REGISTER_TOMIN   0x21
#define MLX90614_REGISTER_PWMCTRL 0x22
#define MLX90614_REGISTER_TARANGE 0x23
#define MLX90614_REGISTER_KE      0x24
#define MLX90614_REGISTER_CONFIG  0x25
#define MLX90614_REGISTER_ADDRESS 0x2E
#define MLX90614_REGISTER_ID0     0x3C
#define MLX90614_REGISTER_ID1     0x3D
#define MLX90614_REGISTER_ID2     0x3E
#define MLX90614_REGISTER_ID3     0x3F
#define MLX90614_REGISTER_SLEEP   0xFF // Not really a register, but close enough

#define I2C_READ_TIMEOUT 1000

typedef enum {
	TEMP_RAW,
	TEMP_K,
	TEMP_C,
	TEMP_F
} temperature_units;

uint8_t _MLX_deviceAddress, _MLX_deviceAddressW,_MLX_deviceAddressR; // MLX90614's 7-bit I2C address
I2C_HandleTypeDef* _MLX_i2cPort;
temperature_units _MLX_defaultUnit; // Keeps track of configured temperature unit

// These keep track of the raw temperature values read from the sensor:
int16_t _MLX_rawAmbient, _MLX_rawObject, _MLX_rawObject2, _MLX_rawMax, _MLX_rawMin;
uint16_t _MLX_id[4]; // Keeps track of the 64-bit ID value




void MLX_IRTherm();
bool MLX_begin(I2C_HandleTypeDef* i2cP);
bool MLX_beginCAddr(I2C_HandleTypeDef* i2cP, uint8_t address);
bool MLX_isConnected();
void MLX_setUnit(temperature_units unit);
bool MLX_read();
float MLX_object();
float MLX_ambient();
float MLX_readEmissivity();
uint8_t MLX_setEmissivity(float emis);
uint8_t MLX_readAddress();
bool MLX_setAddress(uint8_t newAdd);
uint8_t MLX_readID();
uint32_t MLX_getIDH();
uint32_t MLX_getIDL();
bool MLX_readRange();
float MLX_minimum();
float MLX_maximum();
uint8_t MLX_setMax(float maxTemp);
uint8_t MLX_setMin(float minTemp);
void MLX_sleep();
void MLX_wake();
bool MLX_readObject();
bool MLX_readObject2();
bool MLX_readAmbient();
bool MLX_readMax();
bool MLX_readMin();
float MLX_calcTemperature(int16_t rawTemp);
int16_t MLX_calcRawTemp(float calcTemp);
bool MLX_writeEEPROM(uint8_t reg, int16_t data);
bool MLX_I2CReadWord(uint8_t reg, int16_t * dest);
HAL_StatusTypeDef MLX_I2CWriteWord(uint8_t reg, int16_t data);
uint8_t MLX_crc8 (uint8_t inCrc, uint8_t inData);


#endif /* SRC_MLX90614_H_ */


/*
 * MLX_IRTherm();
 * MLX_begin(&hi2c1);
 * MLX_setUnit(TEMP_F);
 * MLX_read();
 * MLX_Object();
 * MLX_ambient();
 */
