/****************************************************************************** 
MLX90614_Get_ID.ino
Print ID register values stored in the MLX90614

This example reads from the MLX90614's ID registers, and 
prints out the 64-byte value to the serial monitor.

Hardware Hookup (if you're not using the eval board):
MLX90614 ------------- Arduino
  VDD ------------------ 3.3V
  VSS ------------------ GND
  SDA ------------------ SDA (A4 on older boards)
  SCL ------------------ SCL (A5 on older boards)
  
Jim Lindblom @ SparkFun Electronics
October 23, 2015
https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library

Development environment specifics:
Arduino 1.6.5
SparkFun IR Thermometer Evaluation Board - MLX90614
******************************************************************************/

#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> //Click here to get the library: http://librarymanager/All#Qwiic_IR_Thermometer by SparkFun
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD


//SerLCD lcd; // Initialize the library with default I2C address 0x72
  IRTherm therm; // Create an IRTherm object to interact with throughout

void setup() 
{
  Serial.begin(115200); // Initialize Serial to log output
  Wire.begin(); //Joing I2C bus
  
  if (therm.begin() == false){ // Initialize thermal IR sensor
    Serial.println("Qwiic IR thermometer did not acknowledge! Freezing!");
    while(1);
  }
  Serial.println("Qwiic IR Thermometer did acknowledge.");
  
  therm.setUnit(TEMP_F); // Set the library's units to Farenheit
  // Alternatively, TEMP_F can be replaced with TEMP_C for Celsius or
  // TEMP_K for Kelvin.
  
  pinMode(LED_BUILTIN, OUTPUT); // LED pin as output

  //LCD
//  lcd.begin(Wire); //Set up the LCD for I2C communication

//  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
//  lcd.setContrast(0); //Set contrast. Lower to 0 for higher contrast.

//  lcd.clear(); //Clear the display - this moves the cursor to home position as well
//  lcd.print("Hello");

  therm.read();
}

void loop() 
{
  digitalWrite(LED_BUILTIN, HIGH);
    
  // Call therm.read() to read object and ambient temperatures from the sensor.
//  if (therm.read()) // On success, read() will return 1, on fail 0.
//  {
    // Use the object() and ambient() functions to grab the object and ambient
  // temperatures.
  // They'll be floats, calculated out to the unit you set with setUnit().
//    Serial.print("Object: " + String(therm.object(), 2));
//    Serial.println("F");
//    Serial.print("Ambient: " + String(therm.ambient(), 2));
//    Serial.println("F");
//    Serial.println();

  // LCD
  // Set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  
//  if (therm.object() > 77){
//    lcd.setFastBacklight(255, 0, 0); //Set backlight to bright white
//    lcd.setCursor(0, 1);
    // Print the number of seconds since reset:
//    lcd.print("Temperature: " + String(therm.object(), 2) + "F");
//    lcd.setCursor(0,0);
//    lcd.print("Ambient: " + String(therm.ambient(), 2) + "F");
//  }
//  else {
//    lcd.setFastBacklight(0,255,0);
//    lcd.setCursor(0, 1);
     // Print the number of seconds since reset:
//    lcd.print("Temperature: " + String(therm.object(), 2) + "F");
//    lcd.setCursor(0,0);
//    lcd.print("Ambient: " + String(therm.ambient(), 2) + "F");
//  }


    
//  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
