/****************************************************************************** 
******************************************************************************/

#include <Wire.h> // I2C library, required for MLX90614
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD


SerLCD lcd; // Initialize the library with default I2C address 0x72
bool seitch = 0;

void setup() 
{
  
  Serial.begin(115200); // Initialize Serial to log output
  Wire.begin(); //Joing I2C bus
  
  pinMode(LED_BUILTIN, OUTPUT); // LED pin as output

  //LCD
  lcd.begin(Wire); //Set up the LCD for I2C communication

  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  lcd.setContrast(0); //Set contrast. Lower to 0 for higher contrast.

  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  lcd.print("Everything will be okay");
  lcd.setCursor(0, 1);
  
}

void loop() 
{
  lcd.setFastBacklight(255,0,0);
  delay(1000);
  lcd.setFastBacklight(0,255,0);
  delay(1000);
  lcd.setFastBacklight(0,0,255);
  delay(1000);
  lcd.setFastBacklight(0,0,0);
  if (seitch){
    lcd.clear();
    lcd.print("Good Luck!");
  }
  else {
     lcd.clear();
    lcd.print("Everything will be okay");
  }
  seitch = !seitch;
}
