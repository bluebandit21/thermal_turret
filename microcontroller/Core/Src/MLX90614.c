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
void MLX_IRTherm()
{
	// Set initial values for all private member variables
	_MLX_deviceAddress = 0;
	_MLX_defaultUnit = TEMP_C;
	_MLX_rawObject = 0;
	_MLX_rawAmbient = 0;
	_MLX_rawObject2 = 0;
	_MLX_rawMax = 0;
	_MLX_rawMin = 0;
}

/* Initializes the library and sets the MLX90614 to the
 * default 7-bit I2C address
 * I2C_HandleTypeDef* i2cP - The I2C port to use
 */
bool MLX_begin(I2C_HandleTypeDef* i2cP){
	_MLX_deviceAddress = MLX90614_DEFAULT_ADDRESS; // Store the address in a private member
	_MLX_i2cPort = i2cP;
	_MLX_defaultUnit = TEMP_RAW;
	return (MLX_isConnected());
}

/* Initializes the library, and prepares
 * communication with an MLX90614 device at the specified 7-bit I2C
 * address.
 * I2C_HandleTypeDef* i2cP - The I2C port to use
 * uint8_t address - The custom Address to be used for the MLX90614
 */
bool MLX_beginCAddr(I2C_HandleTypeDef* i2cP, uint8_t address){

	_MLX_deviceAddress = address;
	_MLX_i2cPort = i2cP;
	_MLX_defaultUnit = TEMP_RAW;
	return (MLX_isConnected());
}

/*
 *  Returns a bool if the temperature sensor is connected
 */
bool MLX_isConnected(){
	uint8_t buf[1];
	buf[0] = MLX90614_REGISTER_ADDRESS;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_MLX_i2cPort, _MLX_deviceAddress, buf, 1, HAL_MAX_DELAY);
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
void MLX_setUnit(temperature_units unit){
	_MLX_defaultUnit = unit; // Store the unit into a private member
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
	return MLX_calcTemperature(_MLX_rawObject);
}

/* ambient() returns the MLX90614's most recently read ambient temperature
 * after the read() function has returned successfully. The float value
 * returned will be in the units specified by setUnit().
 */
float MLX_ambient(){
	// Return the calculated ambient temperature
	return MLX_calcTemperature(_MLX_rawAmbient);
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

		// Write the new address back to EEPROM:
		return MLX_writeEEPROM(MLX90614_REGISTER_ADDRESS, tempAdd);
	}
	return 0;
}

/* readID() reads the 64-bit ID of the MLX90614.
 * Return value is either 1 on success or 0 on failure.
 */
uint8_t MLX_readID(){
	for (int i=0; i<4; i++)
	{
		int16_t temp = 0;
		// Read from all four ID registers, beginning at the first:
		if (!MLX_I2CReadWord(MLX90614_REGISTER_ID0 + i, &temp))
			return 0;
		// If the read succeeded, store the ID into the id array:
		_MLX_id[i] = (uint16_t)temp;
	}
	return 1;
}

/* After calling readID(), getIDH() and getIDL() can be called to read
 * the upper 4 bytes and lower 4-bytes, respectively, of the MLX90614's
 * identification registers.
 */
uint32_t MLX_getIDH(){
	// Return the upper 32 bits of the ID
	return ((uint32_t)_MLX_id[3] << 16) | _MLX_id[2];
}

/* After calling readID(), getIDH() and getIDL() can be called to read
 * the upper 4 bytes and lower 4-bytes, respectively, of the MLX90614's
 * identification registers.
 */
uint32_t MLX_getIDL(){
	// Return the lower 32 bits of the ID
	return ((uint32_t)_MLX_id[1] << 16) | _MLX_id[0];
}

/* readRange() pulls the object maximum and minimum values stored in the
 * MLX90614's EEPROM.
 * It will return either 1 on success or 0 on failure.
 */
bool MLX_readRange() {
	// Read both minimum and maximum values from EEPROM
	if (MLX_readMin() && MLX_readMax())
	{
		// If the read succeeded, return success
		return true;
	}
	return false; // Else return fail
}

/* minimum() and maximum() return the MLX90614's minimum and maximum object
 * sensor readings.
 * The float values returned will be in the units specified by setUnit().
 */
float MLX_minimum(){
	// Return the calculated minimum temperature
	return MLX_calcTemperature(_MLX_rawMin);
}



/* minimum() and maximum() return the MLX90614's minimum and maximum object
 * sensor readings.
 * The float values returned will be in the units specified by setUnit().
 */
float MLX_maximum(){
	// Return the calculated maximum temperature
	return MLX_calcTemperature(_MLX_rawMax);
}


/* setMax(<maxTemp>) and setMin(<minTemp>) configure the MLX90614's
 * maximum and minimum object sensor temperatures.
 */
uint8_t MLX_setMax(float maxTemp){
	// Convert the unit-ed value to a raw ADC value:
	int16_t rawMax = MLX_calcRawTemp(maxTemp);
	// Write that value to the TOMAX EEPROM address:
	return MLX_writeEEPROM(MLX90614_REGISTER_TOMAX, rawMax);
}

/* setMax(<maxTemp>) and setMin(<minTemp>) configure the MLX90614's
 * maximum and minimum object sensor temperatures.
 */
uint8_t MLX_setMin(float minTemp){
	// Convert the unit-ed value to a raw ADC value:
	int16_t rawMin = MLX_calcRawTemp(minTemp);
	// Write that value to the TOMIN EEPROM address:
	return MLX_writeEEPROM(MLX90614_REGISTER_TOMIN, rawMin);
}

/*
 * WARNING:This method is not set up at the moment
 * MLX_sleep() sets the MLX90614 into a low-power sleep mode.
 */
void MLX_sleep(){
	// Calculate a crc8 value.
	// Bits sent: _MLX_deviceAddress (shifted left 1) + 0xFF
	uint8_t crc = MLX_crc8(0, (_MLX_deviceAddress << 1));
	crc = MLX_crc8(crc, MLX90614_REGISTER_SLEEP);

	uint8_t buf[2];
	buf[0] = MLX90614_REGISTER_SLEEP;
	buf[1] = crc;

	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_MLX_i2cPort, _MLX_deviceAddress, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: MLX90614: Sleep command failed: %x\r\n", ret);
	}

	// Set the SCL pin LOW, and SDA pin HIGH (should be pulled up)
	//pinMode(SCL, OUTPUT);
	//digitalWrite(SCL, LOW);
	//pinMode(SDA, INPUT);
}

/*
 * WARNING:This method is not set up at the moment
 * MLX_wake() should revive the MLX90614 from low-power sleep mode.
 */
void MLX_wake(){
	// Wake operation from datasheet
//	_i2cPort->endTransmission(true); // stop i2c bus transmission BEFORE sending wake up request
//	pinMode(SCL, INPUT); // SCL high
//	pinMode(SDA, OUTPUT);
//	digitalWrite(SDA, LOW); // SDA low
//	delay(50); // delay at least 33ms
//	pinMode(SDA, INPUT); // SDA high
//	delay(250);
//	// PWM to SMBus mode:
//	pinMode(SCL, OUTPUT);
//	digitalWrite(SCL, LOW); // SCL low
//	delay(10); // Delay at least 1.44ms
//	pinMode(SCL, INPUT); // SCL high
//	_i2cPort->beginTransmission(_deviceAddress); // reactivate i2c bus transmission AFTER sending wake up request
}


/* These functions individually read the object, object2, and ambient
 * temperature values from the MLX90614's RAM:
 */
bool MLX_readObject(){
	int16_t rawObj;
	// Read from the TOBJ1 register, store into the rawObj variable
	if (MLX_I2CReadWord(MLX90614_REGISTER_TOBJ1, &rawObj))
	{
		// If the read succeeded
		if (rawObj & 0x8000) // If there was a flag error
		{
			return 0; // Return fail
		}
		// Store the object temperature into the class variable
		_MLX_rawObject = rawObj;
		return true;
	}
	return false;
}

/* These functions individually read the object, object2, and ambient
 * temperature values from the MLX90614's RAM:
 */
bool readObject2(){
	int16_t rawObj;
	// Read from the TOBJ2 register, store into the rawObj variable
	if (MLX_I2CReadWord(MLX90614_REGISTER_TOBJ2, &rawObj))
	{
		// If the read succeeded
		if (rawObj & 0x8000) // If there was a flag error
		{
			return 0; // Return fail
		}
		// Store the object2 temperature into the class variable
		_MLX_rawObject2 = rawObj;
		return true;
	}
	return false;
}

/* These functions individually read the object, object2, and ambient
 * temperature values from the MLX90614's RAM:
 */
bool MLX_readAmbient(){
	int16_t rawAmb;
	// Read from the TA register, store value in rawAmb
	if (MLX_I2CReadWord(MLX90614_REGISTER_TA, &rawAmb))
	{
		// If the read succeeds, store the read value
		_MLX_rawAmbient = rawAmb; // return success
		return true;
	}
	return false; // else return fail
}

















/* calcTemperature converts a raw ADC temperature reading to the
 * set unit.
 */
float MLX_calcTemperature(int16_t rawTemp){
	float retTemp;

	if (_MLX_defaultUnit == TEMP_RAW)
	{
		retTemp = (float) rawTemp;
	}
	else
	{
		retTemp = ((float)rawTemp) * 0.02;
		if (_MLX_defaultUnit != TEMP_K)
		{
			retTemp -= 273.15;
			if (_MLX_defaultUnit == TEMP_F)
			{
				retTemp = retTemp * 9.0 / 5.0 + 32;
			}
		}
	}

	return retTemp;
}
















/* Abstract function to read and write 16-bit values from a RAM
 * or EEPROM address in the MLX90614
 */
bool MLX_I2CReadWord(uint8_t reg, int16_t * dest){
	uint8_t buf[3];
	buf[0] = reg;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_MLX_i2cPort, _MLX_deviceAddress, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: MLX90614: MLX_I2CReadWord()-Write: %x\r\n", ret);
	}
	ret = HAL_I2C_Master_Receive(_MLX_i2cPort, _MLX_deviceAddress, buf, 3, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: MLX90614: MLX_I2CReadWord()-Read: %x\r\n", ret);
	}


	// MIGHT READ BACKWARDS
	uint8_t lsb = buf[0];
	uint8_t msb = buf[1];
	uint8_t pec = buf[2];

	uint8_t crc = MLX_crc8(0, (_MLX_deviceAddress << 1));
	crc = MLX_crc8(crc, reg);
	crc = MLX_crc8(crc, (_MLX_deviceAddress << 1) + 1);
	crc = MLX_crc8(crc, lsb);
	crc = MLX_crc8(crc, msb);

	if (crc == pec)
	{
		*dest = (msb << 8) | lsb;
		return true;
	}
	else
	{
		return false;;
	}
}

/* Abstract function to read and write 16-bit values from a RAM
 * or EEPROM address in the MLX90614
 */
uint8_t MLX_I2CWriteWord(uint8_t reg, int16_t data){
	uint8_t crc;
	uint8_t lsb = data & 0x00FF;
	uint8_t msb = (data >> 8);

	crc = MLX_crc8(0, (_MLX_deviceAddress << 1));
	crc = MLX_crc8(crc, reg);
	crc = MLX_crc8(crc, lsb);
	crc = MLX_crc8(crc, msb);

	uint8_t buf[4];
	buf[0] = reg;
	buf[1] = lsb;
	buf[2] = msb;
	buf[3] = crc;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_MLX_i2cPort, _MLX_deviceAddress, buf, 4, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: MLX90614: MLX_I2CWriteWord(): %x\r\n", ret);
	}

	return ret;
}





/* crc8 returns a calculated crc value given an initial value and
 * input data.
 * It's configured to calculate the CRC using a x^8+x^2+x^1+1 poly
 */
uint8_t MLX_crc8 (uint8_t inCrc, uint8_t inData){
	uint8_t i;
	uint8_t data;
	data = inCrc ^ inData;
	for ( i = 0; i < 8; i++ )
	{
		if (( data & 0x80 ) != 0 )
		{
			data <<= 1;
			data ^= 0x07;
		}
		else
		{
			data <<= 1;
		}
	}
	return data;
}

