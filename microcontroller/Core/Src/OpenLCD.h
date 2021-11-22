/*
 * OpenLCD.h
 *
 *  Created on: Nov 24, 2021
 *      Author: AJEKA
 */

#ifndef SRC_OPENLCD_H_
#define SRC_OPENLCD_H_

#include "main.h"


#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

//------------------------LCD DRIVERS---------------------------------------


#define DISPLAY_ADDRESS1W 0xE4 //This is the default address of the OpenLCD
#define DISPLAY_ADDRESS1R 0xE5
#define MAX_ROWS 4
#define MAX_COLUMNS 20

//OpenLCD command characters
#define SPECIAL_COMMAND 254  //Magic number for sending a special command
#define SETTING_COMMAND 0x7C //124, |, the pipe character: The command to change settings: baud, lines, width, backlight, splash, etc

//OpenLCD commands
#define CLEAR_COMMAND 0x2D					//45, -, the dash character: command to clear and home the display
#define CONTRAST_COMMAND 0x18				//Command to change the contrast setting
#define ADDRESS_COMMAND 0x19				//Command to change the i2c address
#define SET_RGB_COMMAND 0x2B				//43, +, the plus character: command to set backlight RGB value
#define ENABLE_SYSTEM_MESSAGE_DISPLAY 0x2E  //46, ., command to enable system messages being displayed
#define DISABLE_SYSTEM_MESSAGE_DISPLAY 0x2F //47, /, command to disable system messages being displayed
#define ENABLE_SPLASH_DISPLAY 0x30			//48, 0, command to enable splash screen at power on
#define DISABLE_SPLASH_DISPLAY 0x31			//49, 1, command to disable splash screen at power on
#define SAVE_CURRENT_DISPLAY_AS_SPLASH 0x0A //10, Ctrl+j, command to save current text on display as splash

// special commands
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00


static uint8_t _i2cAddrW = DISPLAY_ADDRESS1W;
static uint8_t _i2cAddrR = DISPLAY_ADDRESS1R;
static uint8_t _displayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
static uint8_t _displayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
static I2C_HandleTypeDef* _i2cPort = NULL;   //The generic connection to user's chosen I2C hardware
static uint8_t _csPin = 10;

void OpenLCD_init();
void OpenLCD_beginCAddr(uint8_t i2c_addr, I2C_HandleTypeDef *i2c_handler);
void OpenLCD_begin(I2C_HandleTypeDef *i2c_handler);
void OpenLCD_command(uint8_t command);
void OpenLCD_specialCommandMUl(uint8_t command, uint8_t count);
void OpenLCD_specialCommand(uint8_t command);



void OpenLCD_clear();
void OpenLCD_home();
void OpenLCD_setCursor(uint8_t col, uint8_t row);
void OpenLCD_createChar(uint8_t location, uint8_t charmap[]);
void OpenLCD_writeChar(uint8_t location);
size_t OpenLCD_write(uint8_t);
size_t OpenLCD_writebuff(const uint8_t *buffer, size_t size);
size_t OpenLCD_writestr(const char *str);
void OpenLCD_print(const char *str);
void OpenLCD_noDisplay();
void OpenLCD_display();
void OpenLCD_noCursor();
void OpenLCD_cursor();
void OpenLCD_noBlink();
void OpenLCD_blink();
void OpenLCD_scrollDisplayLeft();
void OpenLCD_scrollDisplayRight();
void OpenLCD_scrollDisplayLeftMul(uint8_t count);
void OpenLCD_scrollDisplayRightMul(uint8_t count);
void OpenLCD_moveCursorLeft();
void OpenLCD_moveCursorRight();
void OpenLCD_moveCursorLeftMUl(uint8_t count);
void OpenLCD_moveCursorRightMul(uint8_t count);
void OpenLCD_setBacklight(unsigned long rgb);
void OpenLCD_setBacklightrgb(uint8_t r, uint8_t g, uint8_t b);
void OpenLCD_setFastBacklight(unsigned long rgb);
void OpenLCD_setFastBacklightrgb(uint8_t r, uint8_t g, uint8_t b);
void OpenLCD_leftToRight();
void OpenLCD_rightToLeft();
void OpenLCD_autoscroll();
void OpenLCD_noAutoscroll();
void OpenLCD_setContrast(uint8_t new_val);
void OpenLCD_setAddress(uint8_t new_addr);

void OpenLCD_enableSystemMessages();
void OpenLCD_disableSystemMessages();
void OpenLCD_enableSplash();
void OpenLCD_disableSplash();
void OpenLCD_saveSplash();
uint8_t OpenLCD_getAddress();

#endif /* SRC_OPENLCD_H_ */
