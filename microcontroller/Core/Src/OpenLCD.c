/*
 * LCD library for SparkFun RGB 3.3v Serial Open LCD display
 * with an attached Qwiic adapter.
 *
 *By: Andrew Kent
 *Date: November 24, 2021
 *
 * This library is based on code written By:
 *
 * By: Gaston R. Williams
 * Date: August 22, 2018
 * Update: March 23, 2020 - fixed missing return value in write(uint8_t)
 *
 * License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 *
 * This library is based heavily on the LiquidCrystal_I2C library and the sample code provided with
 * the SparkFun Serial OpenLCD display.  The original LiquidCrystal library by David A. Mellis and
 * modified by Limor Fried and the OpenLCD code by Nathan Seidle at SparkFun.
 *
 * The LiquidCrystal_I2C library was based on the work by DFRobot.
 * (That's the only attribution I found in the code I have. If anyone can provide better information,
 * Plese let me know and I'll be happy to give credit where credit is due.)
 *
 * Original information copied from OpenLCD:
 *
 * The OpenLCD display information is based based on code by
 * Nathan Seidle
 * SparkFun Electronics
 * Date: April 19th, 2015
 *
 * License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 * OpenLCD gives the user multiple interfaces (serial, I2C, and SPI) to control an LCD. SerLCD was the original
 * serial LCD from SparkFun that ran on the PIC 16F88 with only a serial interface and limited feature set.
 * This is an updated serial LCD.
 *
 * Please Note: 0x72 is the 7-bit I2C address. If you are using a different language than Arduino you will probably
 * need to add the Read/Write bit to the end of the address. This means the default read address for the OpenLCD
 * is 0b.1110.0101 or 0xE5 and the write address is 0b.1110.0100 or 0xE4.
 * For more information see https://learn.sparkfun.com/tutorials/i2c
 * Note: This code expects the display to be listening at the default I2C address. If your display is not at 0x72, you can
 * do a hardware reset. Tie the RX pin to ground and power up OpenLCD. You should see the splash screen
 * then "System reset Power cycle me" and the backlight will begin to blink. Now power down OpenLCD and remove
 * the RX/GND jumper. OpenLCD is now reset.
 *
 * To get this code to work, attach a Qwiic adapter to an OpenLCD. Use the Qwiic cable to attach adapter to a SparkFun Blackboard or
 * an Arduino Uno with the Qwiic shield.
 *
 * The OpenLCD has 4.7k pull up resistors on SDA and SCL. If you have other devices on the
 * I2C bus then you may want to disable the pull up resistors by clearing the PU (pull up) jumper.

 * OpenLCD will work at 400kHz Fast I2C. Use the .setClock() call shown below to set the data rate
 * faster if needed.
 * Command cheat sheet:
 * ASCII / DEC / HEX
 * '|'    / 124 / 0x7C - Put into setting mode
 * Ctrl+c / 3 / 0x03 - Change width to 20
 * Ctrl+d / 4 / 0x04 - Change width to 16
 * Ctrl+e / 5 / 0x05 - Change lines to 4
 * Ctrl+f / 6 / 0x06 - Change lines to 2
 * Ctrl+g / 7 / 0x07 - Change lines to 1
 * Ctrl+h / 8 / 0x08 - Software reset of the system
 * Ctrl+i / 9 / 0x09 - Enable/disable splash screen
 * Ctrl+j / 10 / 0x0A - Save currently displayed text as splash
 * Ctrl+k / 11 / 0x0B - Change baud to 2400bps
 * Ctrl+l / 12 / 0x0C - Change baud to 4800bps
 * Ctrl+m / 13 / 0x0D - Change baud to 9600bps
 * Ctrl+n / 14 / 0x0E - Change baud to 14400bps
 * Ctrl+o / 15 / 0x0F - Change baud to 19200bps
 * Ctrl+p / 16 / 0x10 - Change baud to 38400bps
 * Ctrl+q / 17 / 0x11 - Change baud to 57600bps
 * Ctrl+r / 18 / 0x12 - Change baud to 115200bps
 * Ctrl+s / 19 / 0x13 - Change baud to 230400bps
 * Ctrl+t / 20 / 0x14 - Change baud to 460800bps
 * Ctrl+u / 21 / 0x15 - Change baud to 921600bps
 * Ctrl+v / 22 / 0x16 - Change baud to 1000000bps
 * Ctrl+w / 23 / 0x17 - Change baud to 1200bps
 * Ctrl+x / 24 / 0x18 - Change the contrast. Follow Ctrl+x with number 0 to 255. 120 is default.
 * Ctrl+y / 25 / 0x19 - Change the TWI address. Follow Ctrl+x with number 0 to 255. 114 (0x72) is default.
 * Ctrl+z / 26 / 0x1A - Enable/disable ignore RX pin on startup (ignore emergency reset)
 * '+'    / 43 / 0x2B - Set RGB backlight with three following bytes, 0-255
 * ','    / 44 / 0x2C - Display current firmware version
 * '-'    / 45 / 0x2D - Clear display. Move cursor to home position.
 * '.'    / 46 / 0x2E - Enable system messages (ie, display 'Contrast: 5' when changed)
 * '/'    / 47 / 0x2F - Disable system messages (ie, don't display 'Contrast: 5' when changed)
 * '0'    / 48 / 0x30 - Enable splash screen
 * '1'    / 49 / 0x31 - Disable splash screen
 *        / 128-157 / 0x80-0x9D - Set the primary backlight brightness. 128 = Off, 157 = 100%.
 *        / 158-187 / 0x9E-0xBB - Set the green backlight brightness. 158 = Off, 187 = 100%.
 *        / 188-217 / 0xBC-0xD9 - Set the blue backlight brightness. 188 = Off, 217 = 100%.
 *		For example, to change the baud rate to 115200 send 124 followed by 18.
 *
 */
#include "OpenLCD.h"
#include "main.h"
#include "math.h"
#include "stdio.h"

void delay(uint32_t n) {
	HAL_Delay(n);
}

/*
 * Send a command to the display.
 * Used by other functions.
 *
 * byte command to send
 */
void OpenLCD_command(uint8_t command){
	uint8_t buf[2];
	buf[0] = SETTING_COMMAND;
	buf[1] = command;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: Failure LCD: command(%x)\r\n", command);
	}
	delay(10); //Hang out for a bit
}

/*
 * Send a special command to the display.  Used by other functions.
 *
 * byte command to send
 */
void OpenLCD_specialCommand(uint8_t command){
	uint8_t buf[2];
	buf[0] = SPECIAL_COMMAND;
	buf[1] = command;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: Failure LCD: specialCommand(%x)\r\n", command);
	}
	delay(50); //Wait a bit longer for special display commands
}


/*
 * Send multiple special commands to the display.
 * Used by other functions.
 *
 * byte command to send
 * byte count number of times to send
 */
void OpenLCD_specialCommandMul(uint8_t command, uint8_t count)
{
  uint8_t buf[511];
  int n = 0;
  for (int i = 0; i < count; i++)
  {
    buf[n++] = SPECIAL_COMMAND; //Send special command character
    buf[n++] = command;         //Send command code
  }                            // for
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, strlen((char*)buf), HAL_MAX_DELAY);
  if (ret != HAL_OK){
  	printf("ERROR: Failure LCD: specialCommandMul(%x, %i)\r\n", command, count);
  }
  delay(50); //Wait a bit longer for special display commands
}

/*
 * Initialize the display
 *
 */
void OpenLCD_init(){
	OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
	OpenLCD_specialCommand(LCD_ENTRYMODESET | _displayMode);
	OpenLCD_clear();
	delay(50);
}

/*
 * Set up the i2c communication with the SerLCD.
 */
void OpenLCD_begin(I2C_HandleTypeDef *i2c_handler)
{
  _i2cPort = i2c_handler;
  //Call init function since display may have been left in unknown state
  OpenLCD_init();
} // begin

/*
 * Set up the i2c communication with the SerLCD.
 * wirePort - TwoWire port
 * ic2_addr - I2C address
 */
void OpenLCD_beginCAddr(uint8_t i2c_addr, I2C_HandleTypeDef *i2c_handler)
{
  _i2cAddrW = (i2c_addr << 1);
  _i2cAddrR = (i2c_addr << 1) | 1;

  OpenLCD_begin(i2c_handler);
} // begin


/*
 * Send the clear command to the display.  This clears the
 * display and forces the cursor to return to the beginning
 * of the display.
 */
void OpenLCD_clear(I2C_HandleTypeDef* i2c){
	OpenLCD_command(CLEAR_COMMAND);
	delay(10); // a little extra delay after clear
}

/*
 * Send the home command to the display.  This returns the cursor
 * to return to the beginning of the display, without clearing
 * the display.
 */
void OpenLCD_home()
{
	OpenLCD_specialCommand(LCD_RETURNHOME);
}

/*
 * Set the cursor position to a particular column and row.
 *
 * column - byte 0 to 19
 * row - byte 0 to 3
 *
 */
void OpenLCD_setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = {0x00, 0x40, 0x14, 0x54};

  //keep variables in bounds
  //Explicitly cast numeric literals to type byte to avoid ESP32 and ESP8266 compile errors
  row = MAX((uint8_t) 0, row);            //row cannot be less than 0
  row = MIN(row, (uint8_t)(MAX_ROWS - 1)); //row cannot be greater than max rows

  //send the command
  OpenLCD_specialCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
} // setCursor


/*
 * Create a customer character
 * byte   location - character number 0 to 7
 * byte[] charmap  - byte array for character
 */
void OpenLCD_createChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7; // we only have 8 locations 0-7
  uint8_t buf[9];

  //Send request to create a customer character
  buf[0] = SETTING_COMMAND; //Put LCD into setting mode
  buf[1] = (27 + location);
  for (int i = 0; i < 8; i++)
  {
    buf[i + 2] = (charmap[i]);
  } // for
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, strlen((char*)buf), HAL_MAX_DELAY);
  if (ret != HAL_OK){
  	 printf("ERROR: Failure LCD: createChar(%x, charmap = %x)\r\n", location, charmap);
  }
  delay(50); //This takes a bit longer
}

/*
 * Write a customer character to the display
 *
 * byte location - character number 0 to 7
 */
void OpenLCD_writeChar(uint8_t location)
{
  location &= 0x7; // we only have 8 locations 0-7

  OpenLCD_command(35 + location);
}

/*
 * Write a byte to the display.
 * Required for Print.
 */
size_t OpenLCD_write(uint8_t b)
{
  uint8_t buf[1];
  buf[0] = b;
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK){
   	 printf("ERROR: Failure LCD: write(%c)\r\n", b);
  }
  delay(10);         // wait a bit
  return 1;
} // write


/*
 * Write a character buffer to the display.
 * Required for Print.
 */
size_t OpenLCD_writebuff(const uint8_t *buffer, size_t size)
{
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buffer, size, HAL_MAX_DELAY);
  if (ret != HAL_OK){
	  printf("ERROR: Failure LCD: write(const uint8_t * %x, %i)\r\n", buffer, size);
  }
  delay(10); // wait a bit
  return size;
} //write

/*
 * Write a string to the display.
 * Required for Print.
 */
size_t OpenLCD_writestr(const char *str)
{
  if (str == NULL)
    return 0;
  return OpenLCD_writebuff((const uint8_t *)str, strlen(str));
}

/*
  * Turn the display off quickly.
  */
void OpenLCD_noDisplay()
{
  _displayControl &= ~LCD_DISPLAYON;
  OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
} // noDisplay

/*
 * Turn the display on quickly.
 */
void OpenLCD_display()
{
  _displayControl |= LCD_DISPLAYON;
  OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
} // display

/*
  * Turn the underline cursor off.
  */
void OpenLCD_noCursor()
{
  _displayControl &= ~LCD_CURSORON;
  OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
} // noCursor

/*
 * Turn the underline cursor on.
 */
void OpenLCD_cursor()
{
  _displayControl |= LCD_CURSORON;
  OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
} // cursor

/*
  * Turn the blink cursor off.
  */
void OpenLCD_noBlink()
{
  _displayControl &= ~LCD_BLINKON;
  OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
} // noBlink

/*
 * Turn the blink cursor on.
 */
void OpenLCD_blink()
{
  _displayControl |= LCD_BLINKON;
  OpenLCD_specialCommand(LCD_DISPLAYCONTROL | _displayControl);
} // blink

/*
 * Scroll the display one character to the left, without
 * changing the text
 */
void OpenLCD_scrollDisplayLeft()
{
	OpenLCD_specialCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
} // scrollDisplayLeft

/*
 * Scroll the display multiple characters to the left, without
 * changing the text
 *
 * count byte - number of characters to scroll
 */
void OpenLCD_scrollDisplayLeftMul(uint8_t count)
{
	OpenLCD_specialCommandMul(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT, count);
} // scrollDisplayLeft

/*
 * Scroll the display one character to the right, without
 * changing the text
 */
void OpenLCD_scrollDisplayRight()
{
	OpenLCD_specialCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
} // scrollDisplayRight

/*
 * Scroll the display multiple characters to the right, without
 * changing the text
 *
 * count byte - number of characters to scroll
 */
void OpenLCD_scrollDisplayRightMul(uint8_t count)
{
	OpenLCD_specialCommandMul(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT, count);
} // scrollDisplayRight

/*
 *  Move the cursor one character to the left.
 */
void OpenLCD_moveCursorLeft()
{
  OpenLCD_specialCommand(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT);
} // moveCursorLeft

/*
 *  Move the cursor multiple characters to the left.
 *
 *  count byte - number of characters to move
 */
void OpenLCD_moveCursorLeftMul(uint8_t count)
{
	OpenLCD_specialCommandMul(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT, count);
} // moveCursorLeft

/*
 *  Move the cursor one character to the right.
 */
void OpenLCD_moveCursorRight()
{
	OpenLCD_specialCommand(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
} // moveCursorRight

/*
 *  Move the cursor multiple characters to the right.
 *
 *  count byte - number of characters to move
 */
void OpenLCD_moveCursorRightMul(uint8_t count)
{
	OpenLCD_specialCommandMul(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT, count);
} // moveCursorRight

/*
 * Use a standard hex rgb value (0x00000000 to 0x00FFFFFF) to set
 * the backlight color.
 *
 * The encoded long value has form (0x00RRGGBB) where RR, GG and BB
 * are red, green, blue byte values in hex.  The remaining two most
 * significant bytes of the long value are ignored.
 *
 * rgb - unsigned long hex encoded rgb value.
 */
void OpenLCD_setBacklight(unsigned long rgb)
{
  // convert from hex triplet to byte values
  uint8_t r = (rgb >> 16) & 0x0000FF;
  uint8_t g = (rgb >> 8) & 0x0000FF;
  uint8_t b = rgb & 0x0000FF;

  OpenLCD_setBacklightrgb(r, g, b);
}

/*
 * Uses a standard rgb byte triplit eg. (255, 0, 255) to
 * set the backlight color.
 */
void OpenLCD_setBacklightrgb(uint8_t r, uint8_t g, uint8_t b)
{
 uint8_t buf[10];
  // map the byte value range to backlight command range
  uint8_t red = 128 + map(r, 0, 255, 0, 29);
  uint8_t green = 158 + map(g, 0, 255, 0, 29);
  uint8_t blue = 188 + map(b, 0, 255, 0, 29);

  //send commands to the display to set backlights
  //Turn display off to hide confirmation messages
  _displayControl &= ~LCD_DISPLAYON;
  buf[0] = SPECIAL_COMMAND; //Send special command character
  buf[1] = LCD_DISPLAYCONTROL | _displayControl;

  //Set the red, green and blue values
  buf[2] = SETTING_COMMAND; //Set red backlight amount
  buf[3] = red;
  buf[4] = SETTING_COMMAND; //Set green backlight amount
  buf[5] = green;
  buf[6] = SETTING_COMMAND; //Set blue backlight amount
  buf[7] = blue;

  //Turn display back on and end
  _displayControl |= LCD_DISPLAYON;
  buf[8] = SPECIAL_COMMAND;                      //Send special command character
  buf[9] = LCD_DISPLAYCONTROL | _displayControl; //Turn display on as before
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 10, HAL_MAX_DELAY);
  if (ret != HAL_OK){
   	 printf("ERROR: Failure LCD: setBacklight(%i, %i, %i)\r\n", r, g, b);
  }
  delay(50);                                      //This one is a bit slow
} // setBacklight

// New backlight function
void OpenLCD_setFastBacklight(unsigned long rgb)
{
  // convert from hex triplet to byte values
  uint8_t r = (rgb >> 16) & 0x0000FF;
  uint8_t g = (rgb >> 8) & 0x0000FF;
  uint8_t b = rgb & 0x0000FF;

  OpenLCD_setFastBacklightrgb(r, g, b);
}

/*
 * Uses a standard rgb byte triplit eg. (255, 0, 255) to
 * set the backlight color.
 * I2C_HandleTypeDef* i2c - LCD address
 * uint8_t r - red color 0-255
 * uint8_t g - green color 0-255
 * uint8_t b - blue color 0-255
 */
void OpenLCD_setFastBacklightrgb(uint8_t r, uint8_t g, uint8_t b){
	uint8_t buf[5];
	buf[0] = SETTING_COMMAND;
	buf[1] = SET_RGB_COMMAND;
	buf[2] = r;
	buf[3] = g;
	buf[4] = b;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 5, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: Failure LCD: setFastBacklight(%i, %i, %i)\r\n", r, g, b);
	}
	delay(10);
}


/*
 * Set the text to flow from left to right.  This is the direction
 * that is common to most Western languages.
 */
void OpenLCD_leftToRight()
{
  _displayMode |= LCD_ENTRYLEFT;
  OpenLCD_specialCommand(LCD_ENTRYMODESET | _displayMode);
} // leftToRight

/*
 * Set the text to flow from right to left.
 */
void OpenLCD_rightToLeft()
{
  _displayMode &= ~LCD_ENTRYLEFT;
  OpenLCD_specialCommand(LCD_ENTRYMODESET | _displayMode);
} //rightToLeft

/*
 * Turn autoscrolling on. This will 'right justify' text from
 * the cursor.
 */
void OpenLCD_autoscroll()
{
  _displayMode |= LCD_ENTRYSHIFTINCREMENT;
  OpenLCD_specialCommand(LCD_ENTRYMODESET | _displayMode);
} //autoscroll

/*
 * Turn autoscrolling off.
 */
void OpenLCD_noAutoscroll()
{
  _displayMode &= ~LCD_ENTRYSHIFTINCREMENT;
  OpenLCD_specialCommand(LCD_ENTRYMODESET | _displayMode);
} //noAutoscroll

/*
 * Change the contrast from 0 to 255. 120 is default.
 * Lower is Darker
 * I2C_HandleTypeDef* i2c - LCD address
 * uint8_t contrast - new contrast value
 */
void OpenLCD_setContrast(uint8_t contrast){
	uint8_t buf[3];
	buf[0] = SETTING_COMMAND;
	buf[1] = CONTRAST_COMMAND;
	buf[2] = contrast;
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 3, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		printf("ERROR: Failure LCD: SetContrast(%i)\r\n", contrast);
	}
	delay(10);
}

/*
 * Change the I2C Address. 0x72 is the default.
 * Note that this change is persistent.  If anything
 * goes wrong you may need to do a hardware reset
 * to unbrick the display.
 *
 * byte new_addr - new i2c address
 */
void OpenLCD_setAddress(uint8_t new_addr)
{
  uint8_t buf[3];

  //send commands to the display to set backlights
  buf[0] = SETTING_COMMAND; //Send Address command
  buf[1] = ADDRESS_COMMAND; //0x19
  buf[2] = new_addr;        //Send new Address value

  //Update our own address so we can still talk to the display
  _i2cAddrW = (new_addr << 1);
  _i2cAddrR = (new_addr << 1) | 1;

  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 3, HAL_MAX_DELAY);
  if (ret != HAL_OK){
  	printf("ERROR: Failure LCD: SetAddress(%x)\r\n", new_addr);
  }
  delay(50); //This may take awhile
  } //setAddress


//Enable system messages
//This allows user to see printing messages like 'UART: 57600' and 'Contrast: 5'
void OpenLCD_enableSystemMessages()
{
  uint8_t buf[2] = {SETTING_COMMAND, ENABLE_SYSTEM_MESSAGE_DISPLAY};
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
  if (ret != HAL_OK){
    printf("ERROR: Failure LCD: enableSystemMessages()\r\n");
  }
  delay(10);
}

//Disable system messages
//This allows user to disable printing messages like 'UART: 57600' and 'Contrast: 5'
void OpenLCD_disableSystemMessages()
{
	uint8_t buf[2] = {SETTING_COMMAND, DISABLE_SYSTEM_MESSAGE_DISPLAY};
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
	  printf("ERROR: Failure LCD: disableSystemMessages()\r\n");
	}
	delay(10);
}

//Enable splash screen at power on
void OpenLCD_enableSplash()
{
	uint8_t buf[2] = {SETTING_COMMAND, ENABLE_SPLASH_DISPLAY};
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
	  printf("ERROR: Failure LCD: enableSplashDisplay()\r\n");
	}
	delay(10);
}

//Disable splash screen at power on
void OpenLCD_disableSplash()
{
	uint8_t buf[2] = {SETTING_COMMAND, DISABLE_SPLASH_DISPLAY};
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
	 printf("ERROR: Failure LCD: disableSplashDisplay()\r\n");
	}
		delay(10);
}

//Save the current display as the splash
void OpenLCD_saveSplash()
{
  //Save whatever is currently being displayed into EEPROM
  //This will be displayed at next power on as the splash screen
  uint8_t buf[2] = {SETTING_COMMAND, SAVE_CURRENT_DISPLAY_AS_SPLASH};
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(_i2cPort, _i2cAddrW, buf, 2, HAL_MAX_DELAY);
  if (ret != HAL_OK){
   printf("ERROR: Failure LCD: saveSplash()\r\n");
  }
  delay(10);
}

/*
 * getAddress
 *
 * Returns private variable I2C address
 */
uint8_t OpenLCD_getAddress()
{
  return (_i2cAddrW >> 1);
} //getAddress
