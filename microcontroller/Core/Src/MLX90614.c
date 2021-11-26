/*
 * MLX90614.cpp
 * Source file for the SparkFun IR Thermometer Library
 *
 * By: Andrew Kent
 *
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

#include "main.h"
#include "MLX90614.h"
#include "float.h"
#include "stdbool.h"

/* Default constructor, does very little besides setting class variable
 * initial values.
 */
void IRTherm()
{
	// Set initial values for all private member variables
	_deviceAddress = 0;
	_defaultUnit = TEMP_C;
	_rawObject = 0;
	_rawAmbient = 0;
	_rawObject2 = 0;
	_rawMax = 0;
	_rawMin = 0;
}

/* Initializes the library and sets the MLX90614 to the
 * default 7-bit I2C address
 * I2C_HandleTypeDef* i2cP - The I2C port to use
 */
bool MLX_begin(I2C_HandleTypeDef* i2cP){
	_deviceAddress = MLX90614_DEFAULT_ADDRESS; // Store the address in a private member
	_i2cPort = i2cP;
	_defaultUnit = TEMP_RAW;
	return (MLX_isConnected());
}

/* Initializes the library, and prepares
 * communication with an MLX90614 device at the specified 7-bit I2C
 * address.
 * I2C_HandleTypeDef* i2cP - The I2C port to use
 * uint8_t address - The custom Address to be used for the MLX90614
 */
bool MLX_beginCAddr(I2C_HandleTypeDef* i2cP, uint8_t address){

	_deviceAddress = address;
	_i2cPort = i2cP;
	_defaultUnit = TEMP_RAW;
	return (MLX_isConnected());
}

/*
 *  Returns a bool if the temperature sensor is connected
 */
bool MLX_isConnected(){
	uint8_t buf[1] = {0};
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _deviceAddress, buf, 0, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		return false;
	}
	return true;
}

/* setUnit(<unit>) configures the units returned by the ambient(),
 * object(), minimum() and maximum() functions, and it determines what
 * units the setMin() and setMax() functions should expect.
 * <unit> can be either:
 * - TEMP_RAW: No conversion, just the raw 12-bit ADC reading
 * - TEMP_K: Kelvin
 * - TEMP_C: Celsius
 * - TEMP_F: Farenheit
 */
void setUnit(temperature_units unit){
	_defaultUnit = unit; // Store the unit into a private member
}

/* read() pulls the latest ambient and object temperatures from the
 *  MLX90614. It will return either 1 on success or 0 on failure. (Failure
 *  can result from either a timed out I2C transmission, or an incorrect
 * checksum value).
 *
 */
bool MLX_read(){
	// read both the object and ambient temperature values
	if (MLX_readObject() && MLX_readAmbient())
	{
		// If the reads succeeded, return success
		return true;
	}
	return false; // Else return fail
}

/* object() returns the MLX90614's most recently read object temperature
 * after the read() function has returned successfully. The float value
 * returned will be in the units specified by setUnit().
 */
float MLX_object(){
	// Return the calculated object temperature
	return MLX_calcTemperature(_rawObject);
}

/* ambient() returns the MLX90614's most recently read ambient temperature
 * after the read() function has returned successfully. The float value
 * returned will be in the units specified by setUnit().
 */
float MLX_ambient(){
	// Return the calculated ambient temperature
	return MLX_calcTemperature(_rawAmbient);
}

/* readEmissivity() reads the MLX90614's emissivity setting. It will
 * return a value between 0.1 and 1.0.
 */
float MLX_readEmissivity(){
	int16_t ke;
	if (MLX_I2CReadWord(MLX90614_REGISTER_KE, &ke))
	{
		// If we successfully read from the ke register
		// calculate the emissivity between 0.1 and 1.0:
		return (((float)((uint16_t)ke)) / 65535.0);
	}
	return 0; // Else return fail
}

/* setEmissivity(<emis>) can set the MLX90614's configured emissivity
 * EEPROM value.
 * The <emis> parameter should be a value between 0.1 and 1.0.
 * The function will return either 1 on success or 0 on failure.
 */
uint8_t MLX_setEmissivity(float emis){
	// Make sure emissivity is between 0.1 and 1.0
	if ((emis > 1.0) || (emis < 0.1)){
		return 0; // Return fail if not
	}
	// Calculate the raw 16-bit value:
	uint16_t ke = (uint16_t)(65535.0 * emis);
	ke = constrain(ke, 0x2000, 0xFFFF);

	// Write that value to the ke register
	return MLX_writeEEPROM(MLX90614_REGISTER_KE, (int16_t)ke);
}

/* readAddress() returns the MLX90614's configured 7-bit I2C bus address.
 * A value between 0x01 and 0x7F should be returned.
 */
uint8_t MLX_readAddress(){
	int16_t tempAdd;
	// Read from the 7-bit I2C address EEPROM storage address:
	if (MLX_I2CReadWord(MLX90614_REGISTER_ADDRESS, &tempAdd))
	{
		// If read succeeded, return the address:
		return (uint8_t) tempAdd;
	}
	return 0; // Else return fail
}

/* setAddress(<newAdd>) can set the MLX90614's 7-bit I2C bus address.
 * The <newAdd> parameter should be a value between 0x01 and 0x7F.
 * The function returns 1 on success and 0 on failure.
 * The new address won't take effect until the device is reset.
 */
bool MLX_setAddress(uint8_t newAdd){
	int16_t tempAdd;
	// Make sure the address is within the proper range:
	if ((newAdd >= 0x80) || (newAdd == 0x00))
		return 0; // Return fail if out of range
	// Read from the I2C address address first:
	if (MLX_I2CReadWord(MLX90614_REGISTER_ADDRESS, &tempAdd))
	{
		tempAdd &= 0xFF00; // Mask out the address (MSB is junk?)
		tempAdd |= newAdd; // Add the new address

		// Write the new addres back to EEPROM:
		return MLX_writeEEPROM(MLX90614_REGISTER_ADDRESS, tempAdd);
	}
	return 0;
}




