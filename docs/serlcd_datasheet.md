# LCD Codes for SparkFun RGB 3.3v Serial Open LCD display 
### with an attached Qwiic adapter

Rearranged for viewing by: Andrew Kent

Original information copied from SerLCD:
By: Gaston R. Williams

Original information copied from OpenLCD:
The OpenLCD display information is based based on code by
Nathan Seidle

## Addresses

Please Note: 0x72 is the 7-bit I2C address. If you are using a different language than Arduino you will probably
need to add the Read/Write bit to the end of the address. This means the default read address for the OpenLCD
is 0b.1110.0101 or 0xE5 and the write address is 0b.1110.0100 or 0xE4.

For more information see https://learn.sparkfun.com/tutorials/i2c
Note: This information expects the display to be listening at the default I2C address. If your display is not at 0x72, you can
do a hardware reset. Tie the RX pin to ground and power up OpenLCD. You should see the splash screen
then "System reset Power cycle me" and the backlight will begin to blink. Now power down OpenLCD and remove
the RX/GND jumper. OpenLCD is now reset.

To get this code to work, attach a Qwiic adapter to an OpenLCD. Use the Qwiic cable to attach adapter to a SparkFun Blackboard or
an Arduino Uno with the Qwiic shield.

 The OpenLCD has 4.7k pull up resistors on SDA and SCL. If you have other devices on the
 I2C bus then you may want to disable the pull up resistors by clearing the PU (pull up) jumper.

OpenLCD will work at 400kHz Fast I2C. Use the .setClock() call shown below to set the data rate
faster if needed.

## Command cheat sheet:

|  ASCII |   DEC   |                                          HEX                                                |
| :----: | :------ | :------------------------------------------------------------------------------------------ |
|  \|    |  124    | 0x7C - Put into setting mode                                                                |
| Ctrl+c |    3    | 0x03 - Change width to 20                                                                   |
| Ctrl+d |    4    | 0x04 - Change width to 16                                                                   |
| Ctrl+e |    5    | 0x05 - Change lines to 4                                                                    |
| Ctrl+f |    6    | 0x06 - Change lines to 2                                                                    |
| Ctrl+g |    7    | 0x07 - Change lines to 1                                                                    |
| Ctrl+h |    8    | 0x08 - Software reset of the system                                                         |
| Ctrl+i |    9    | 0x09 - Enable/disable splash screen                                                         |
| Ctrl+j |   10    | 0x0A - Save currently displayed text as splash                                              |
| Ctrl+k |   11    | 0x0B - Change baud to 2400bps                                                               |
| Ctrl+l |   12    | 0x0C - Change baud to 4800bps                                                               |
| Ctrl+m |   13    | 0x0D - Change baud to 9600bps                                                               |
| Ctrl+n |   14    | 0x0E - Change baud to 14400bps                                                              |
| Ctrl+o |   15    | 0x0F - Change baud to 19200bps                                                              |
| Ctrl+p |   16    | 0x10 - Change baud to 38400bps                                                              |
| Ctrl+q |   17    | 0x11 - Change baud to 57600bps                                                              |
| Ctrl+r |   18    | 0x12 - Change baud to 115200bps                                                             |
| Ctrl+s |   19    | 0x13 - Change baud to 230400bps                                                             |
| Ctrl+t |   20    | 0x14 - Change baud to 460800bps                                                             |
| Ctrl+u |   21    | 0x15 - Change baud to 921600bps                                                             |
| Ctrl+v |   22    | 0x16 - Change baud to 1000000bps                                                            |
| Ctrl+w |   23    | 0x17 - Change baud to 1200bps                                                               |
| Ctrl+x |   24    | 0x18 - Change the contrast. Follow Ctrl+x with number 0 to 255. 120 is default.             |
| Ctrl+y |   25    | 0x19 - Change the TWI address. Follow Ctrl+x with number 0 to 255. 114 (0x72) is default.   |
| Ctrl+z |   26    | 0x1A - Enable/disable ignore RX pin on startup (ignore emergency reset)                     |
|  +     |   43    | 0x2B - Set RGB backlight with three following bytes, 0-255                                  |
|  ,     |   44    | 0x2C - Display current firmware version                                                     |
|  -     |   45    | 0x2D - Clear display. Move cursor to home position.                                         |
|  .     |   46    | 0x2E - Enable system messages (ie, display 'Contrast: 5' when changed)                      |
|  /     |   47    | 0x2F - Disable system messages (ie, don't display 'Contrast: 5' when changed)               |
|  0     |   48    | 0x30 - Enable splash screen                                                                 |
|  1     |   49    | 0x31 - Disable splash screen                                                                |
|        | 128-157 | 0x80-0x9D - Set the primary backlight brightness. 128 = Off, 157 = 100%.                    |
|        | 158-187 | 0x9E-0xBB - Set the green backlight brightness. 158 = Off, 187 = 100%.                      |
|        | 188-217 | 0xBC-0xD9 - Set the blue backlight brightness. 188 = Off, 217 = 100%.                       |

For example, to change the baud rate to 115200 send 124 followed by 18.

## All Commands

```cpp
#define DISPLAY_ADDRESS1 0x72 //This is the default address of the OpenLCD
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

```


## Writing to the Display

```cpp
/*
 * Write a byte to the display.
 * Required for Print.
 */
size_t SerLCD::write(uint8_t b)
{
  beginTransmission(); // transmit to device
  transmit(b);
  endTransmission(); //Stop transmission
  delay(10);         // wait a bit
  return 1;
} // write

/*
 * Write a character buffer to the display.
 * Required for Print.
 */
size_t SerLCD::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  beginTransmission(); // transmit to device
  while (size--)
  {
    transmit(*buffer++);
    n++;
  }                  //while
  endTransmission(); //Stop transmission
  delay(10);         //
  return n;
} //write

/*
 * Write a string to the display.
 * Required for Print.
 */
size_t SerLCD::write(const char *str)
{
  if (str == NULL)
    return 0;
  return write((const uint8_t *)str, strlen(str));
}
```


## Example Functions

```cpp
#define SETTING_COMMAND    0x7C //124, |, the pipe character: The command to change settings: baud, lines, width, backlight, splash, etc
#define SET_RGB_COMMAND    0x2B				//43, +, the plus character: command to set backlight RGB value
#define SPECIAL_COMMAND    254  //Magic number for sending a special command
#define LCD_DISPLAYCONTROL 0x08
#define LCD_ENTRYMODESET   0x04
#define CLEAR_COMMAND      0x2D					//45, -, the dash character: command to clear and home the display

/*
 * Initialize the display
 *
 */
void SerLCD::init()
{
  beginTransmission();
  transmit(SPECIAL_COMMAND);                      //Send special command character
  transmit(LCD_DISPLAYCONTROL | _displayControl); //Send the display command
  transmit(SPECIAL_COMMAND);                      //Send special command character
  transmit(LCD_ENTRYMODESET | _displayMode);      //Send the entry mode command
  transmit(SETTING_COMMAND);                      //Put LCD into setting mode
  transmit(CLEAR_COMMAND);                        //Send clear display command
  endTransmission();                              //Stop transmission
  delay(50);                                      //let things settle a bit
} //init


// New backlight function
void SerLCD::setFastBacklight(unsigned long rgb)
{
  // convert from hex triplet to byte values
  byte r = (rgb >> 16) & 0x0000FF;
  byte g = (rgb >> 8) & 0x0000FF;
  byte b = rgb & 0x0000FF;

  //send commands to the display to set backlights
  beginTransmission();       // transmit to device
  transmit(SETTING_COMMAND); //Send special command character
  transmit(SET_RGB_COMMAND); //Send the set RGB character '+' or plus
  transmit(r);               //Send the red value
  transmit(g);               //Send the green value
  transmit(b);               //Send the blue value
  endTransmission();         //Stop transmission
  delay(10);
} // setFastBacklight






/*
 *  Move the cursor multiple characters to the left.
 *
 *  count byte - number of characters to move
 */
void SerLCD::moveCursorLeft(byte count)
{

  beginTransmission(); // transmit to device

  for (int i = 0; i < count; i++)
  {
    transmit(SPECIAL_COMMAND); //Send special command character
    transmit(0x10);         //Send command code
  }                            // for
  endTransmission();           //Stop transmission

  delay(50); //Wait a bit longer for special display commands
} // moveCursorLeft


/*
 *  Move the cursor one character to the right.
 */
void SerLCD::moveCursorRight()
{
  beginTransmission(); // transmit to device

  for (int i = 0; i < count; i++)
  {
    transmit(SPECIAL_COMMAND); //Send special command character
    transmit(0x14);         //Send command code
  }                            // for
  endTransmission();           //Stop transmission

  delay(50); //Wait a bit longer for special display commands


/*
 * Send the clear command to the display.  This clears the
 * display and forces the cursor to return to the beginning
 * of the display.
 */
void SerLCD::clear()
{
  beginTransmission();       // transmit to device
  transmit(SETTING_COMMAND); //Put LCD into setting mode
  transmit(CLEAR_COMMAND);         //Send the command code
  endTransmission();         //Stop transmission

  delay(10); //Hang out for a bit
  delay(10); // a little extra delay after clear
}

```