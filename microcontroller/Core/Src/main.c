/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "OpenLCD.h"
#include "MLX90614.h"
#include "stdio.h"
#include "float.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//------------------------MISC TYPES------------------------------------------
typedef struct I2C_Module
{
  I2C_HandleTypeDef   instance;
  uint16_t            sdaPin;
  GPIO_TypeDef*       sdaPort;
  uint16_t            sclPin;
  GPIO_TypeDef*       sclPort;
}I2C_Module;



enum SYSTEM_STATE{
	START, /* Nobody centered frame for a while / just powered on. Display
			* nothing on the display, camera heads to (0,0) or tracks towards forehead if present */
	CENTERED, /* We just saw a centered forehead in frame. Take the temperature. If it's not high enough,
	           * tell the user to move closer to the camera on the display */
	TOUCH_WAIT, /* Tell the user to touch the contact sensor and show yellow, along with temperature reading.
				 * Wait for them to touch the contact sensor (i.e. contact sensor goes above ambient)
				 * If they don't after some timeout, move back to RESET*/
	TOUCH_HOLD, /* Have them hodl for a while after detecting initial */
	PAUSE /* Delay for a brief period of time to let the user see the thing, then go to START*/
};



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LASER_SMBUS_ADDRESS 0xB4

#define LASER_TOBJ1_ADDRESS 0x007

#define COMMAND_READ 0x1

//------------------------GIMBAL CONSTANTS-------------------------------------
//Determined experimentally (the datasheet lies)
#define SERVO_PULSE_WIDTH_TICKS 35471
#define SERVO_YAW_NEUTRAL_TICKS  2700
//Range from neutral to +max or -max in ticks
#define SERVO_YAW_RANGE_TICKS 1700 //Determined experimentally
#define SERVO_YAW_MAX_ANGLE 95 //Determined experimentally

#define SERVO_PITCH_NEUTRAL_TICKS  2600 //TODO: Measure better? These still feel slightly off
//Range from neutral to +max or -max in ticks
#define SERVO_PITCH_RANGE_TICKS 1700 //TODO: Measure
#define SERVO_PITCH_MAX_ANGLE 95 //TODO: Measure
#define SERVO_PITCH_MAX_SAFE_ANGLE_MIN 40
#define SERVO_PITCH_MAX_SAFE_ANGLE_MAX -75

//------------------------TOUCH THERMOMETER CONSTANTS-----------------------------
const float R1 = 32.66;  //kOhms
const float R2 = 9.866; //kOhms

//---------------------------VISION INTERFACE CONSTANTS---------------------------

GPIO_TypeDef* VISION_LEFT_BANK = GPIOB;
GPIO_TypeDef* VISION_RIGHT_BANK = GPIOB;
GPIO_TypeDef* VISION_UP_BANK = GPIOB;
GPIO_TypeDef* VISION_DOWN_BANK = GPIOB;
GPIO_TypeDef* VISION_SEES_FACE_BANK = GPIOC;
uint16_t VISION_LEFT_PIN = GPIO_PIN_3;
uint16_t VISION_RIGHT_PIN = GPIO_PIN_4;
uint16_t VISION_UP_PIN = GPIO_PIN_5;
uint16_t VISION_DOWN_PIN = GPIO_PIN_10;
uint16_t VISION_SEES_FACE_PIN = GPIO_PIN_8;



//---------------------------STATE LOGIC CONSTANTS----------------------
#define TEMP_SAFE 78.00 //TODO: Adjust me
#define TEMP_UNSAFE 86.00 //TODO: Adjust me
#define TEMP_THRESHOLD 76.00 //TODO: Adjust me
#define RESET_TIMEOUT 20 //TODO: Adjust me
#define DELAY_TIMEOUT 40 //TODO: Adjust me
#define TOUCH_TEMP_THRESHOLD 73.0 //TODO: Adjust me
#define TOUCH_TIMEOUT 100 //TODO: Adjust me
#define TOUCH_TEMP_SAFE 73.5 //TODO: Adjust me
#define TOUCH_HOLD_COUNT 30 //TODO: Adjust me

const char* MESSAGE_SEARCHING = "Searching...";
const char* MESSAGE_MOVE_CLOSER = "Move closer";
const char* MESSAGE_TOUCH = "Touch sensor";



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//------------------------CONTACT THERMOMETER DRIVERS--------------------------

float read_temperature_blocking();

/* float read_temperature_blocking()
 *  Reports temperature read by contact temperature sensor and returns as a float degrees Fahrenheit
 *  This is a blocking function and will wait several cycles for the ADC to finish converting.
 */
float read_temperature_blocking(){
	  HAL_ADC_Start(&hadc1);//start conversion
	  HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	  float RAW_COUNT = HAL_ADC_GetValue(&hadc1);
	  float voltage = RAW_COUNT * 3.3 / 4096;
	  float corrected_voltage = (voltage * R2 / (R1+R2) ) - 0.5; //Remove DC offset, de scale
	  float temp_c = corrected_voltage * 100;
	  float temp_f = temp_c * 1.8  + 32;

	  return temp_f;
}

//------------------------GIMBAL DRIVERS---------------------------------------

void initialize_gimbal();
int set_gimbal_angles(int yaw, int pitch);


/* void initialize_gimbal()
 *  Initializes hardware timers for gimbals and sets init position to (0,0)
 */
void initialize_gimbal(){
	//Start hardware timers
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

	//Move to starting position
	set_gimbal_angles(0,0);
}

/* int target_gimbal_angles(short yaw, short pitch)
 *  Moves gimbal to specified position of yaw and pitch.
 *  Yaw can range from -95 degrees to 95 degrees
 *  Pitch can range from //TODO: Measure me
 *  If a move would cause physical damage to the gimbal, or make the servos unhappy,
 *   it is not made and -1 is returned as an error status.
 */
int set_gimbal_angles(int yaw, int pitch){
	if((yaw > SERVO_YAW_MAX_ANGLE) || (yaw < -SERVO_YAW_MAX_ANGLE)\
			|| (pitch > SERVO_PITCH_MAX_SAFE_ANGLE_MIN) || (pitch < SERVO_PITCH_MAX_SAFE_ANGLE_MAX)){
		//Refuse to do this move; it'd cause bad things to occur physically
		//TODO: Add logging here
		return -1;
	}
	int yaw_pulse_width = SERVO_YAW_NEUTRAL_TICKS + (yaw * SERVO_YAW_RANGE_TICKS) / SERVO_YAW_MAX_ANGLE;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, yaw_pulse_width);

	int pitch_pulse_width = SERVO_PITCH_NEUTRAL_TICKS + (pitch * SERVO_PITCH_RANGE_TICKS) / SERVO_PITCH_MAX_ANGLE;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pitch_pulse_width);
	return 0;

}


//-----------------------VISION INTERFACE DRIVERS--------------------------------------

GPIO_PinState target_left();
GPIO_PinState target_right();
GPIO_PinState target_up();
GPIO_PinState target_down();
GPIO_PinState sees_face();

GPIO_PinState target_left(){
	return HAL_GPIO_ReadPin(VISION_LEFT_BANK, VISION_LEFT_PIN);
}
GPIO_PinState target_right(){
	return HAL_GPIO_ReadPin(VISION_RIGHT_BANK, VISION_RIGHT_PIN);
}
GPIO_PinState target_up(){
	return HAL_GPIO_ReadPin(VISION_UP_BANK, VISION_UP_PIN);
}
GPIO_PinState target_down(){
	return HAL_GPIO_ReadPin(VISION_DOWN_BANK, VISION_DOWN_PIN);
}

GPIO_PinState sees_face(){
	return HAL_GPIO_ReadPin(VISION_SEES_FACE_BANK, VISION_SEES_FACE_PIN);
}

//-------------------------MISC------------------------------------------------------
void clear_i2c_busy();
void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c);


/* void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c)
 * Does magic to reset the HAL_Busy I2C state when it gets stuck there sometimes.
 * Sourced from https://electronics.stackexchange.com/questions/267972/i2c-busy-flag-strange-behaviour
 */
void I2C_ClearBusyFlagErratum(struct I2C_Module* i2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  i2c->instance.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;
  GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->instance.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&(i2c->instance));
}



/* void clear_i2c_busy()
 *  Sometimes the HAL I2C implementation randomly gets stuck in HAL_BUSY
 *  This fixes it. It is hacky and awful, but so is the HAL library apparently.
 */
void clear_i2c_busy(){
	//This is a horrible hack and shouldn't be neccesary. Yet, it is.
	I2C_Module I2C;
	I2C.instance = hi2c1;
	I2C.sclPin = I2C1_SCL_Pin;
	I2C.sclPort = I2C1_SCL_GPIO_Port;
	I2C.sdaPin = I2C1_SDA_Pin;
	I2C.sdaPort = I2C1_SDA_GPIO_Port;

	I2C_ClearBusyFlagErratum(&I2C);
}

void initialize_lcd(){
	 OpenLCD_begin(&hi2c1);
	 OpenLCD_setContrast(0);
	 OpenLCD_setFastBacklightrgb(128, 128, 128);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  // 4 inputs from Jetson Nano
  //- Up
  //- Down
  //- Left
  //- Right
  //If none are high, laser is aimed properly.

  // 2 PWM Outputs

  // 2 I2C I/O
  // Temp Sens
  // Display

  // ADC for touch

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  initialize_gimbal();
  initialize_lcd();

  clear_i2c_busy();


  MLX_IRTherm();
  bool MLX_OK = MLX_begin(&hi2c1);
  if (!MLX_OK){
	  printf("ERROR: MLX90614: MLX_begin(): Not Connected");
  }
  MLX_setUnit(TEMP_F);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int yaw = 0;
  int pitch = 0;
  long counter = 0;
  float locked_temp;

  enum SYSTEM_STATE state = START;

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  clear_i2c_busy();

	  //--------------------------GIMBAL TRACKING LOGIC: ALWAYS RUNS----------------------

	  if(sees_face()){
		  if(target_left()){
			  printf("Targeting left\r\n");
			  if(yaw < SERVO_YAW_MAX_ANGLE){
				yaw = yaw + 1;
			  }
		  }
		  if(target_right()){
			  printf("Targeting right\r\n");
			  if(yaw > -SERVO_YAW_MAX_ANGLE){
				  yaw = yaw - 1;
			  }
		  }
		  if(target_up()){
			  printf("Targeting up\r\n");
			  if(pitch > SERVO_PITCH_MAX_SAFE_ANGLE_MAX){
				  pitch = pitch - 1;
			  }
		  }
		  if(target_down()){
			  printf("Targeting down\r\n");
			  if(pitch < SERVO_PITCH_MAX_SAFE_ANGLE_MIN){
				  pitch = pitch + 1;
			  }
		  }
	  }else{
		  //Head towards origin (slowly lol)
		  if(pitch > 0){
			  pitch --;
		  }
		  if(pitch < 0){
			  pitch ++;
		  }
		  if(yaw > 0){
			  yaw --;
		  }
		  if(yaw < 0){
			  yaw ++;
		  }
	  }
	  printf("Attempting move to %d,%d\r\n", yaw, pitch);
	  set_gimbal_angles(yaw, pitch);
	  //----------------------STATE MACHINE LOGIC--------------------------
	  switch(state){
	  case START:
		  OpenLCD_clear();
		  OpenLCD_setCursor(0, 0);
		  OpenLCD_writestr(MESSAGE_SEARCHING);
		  if(sees_face() && !(target_down()||target_up()||target_left()||target_right())){
			  //We're centered.
			  printf("Transitioning state to CENTERED\r\n");
			  counter = 0; //We just saw someone
			  state = CENTERED;
		  }
		  break;
	  case CENTERED:
		  if(!sees_face() || (target_down()||target_up()||target_left()||target_right())){
			  counter++;
			  if(counter > RESET_TIMEOUT){
				  OpenLCD_clear();
				  state = START;
			  }
			  OpenLCD_clear();
			  OpenLCD_setCursor(0, 0);
			  OpenLCD_writestr(MESSAGE_SEARCHING);
			  break; //We're not seeing a face; don't even try to read temp
		  }else{
			  counter = 0;
		  }
		  //We're actively centered on a face still
		  //Read the temperature
		  MLX_read();
		  float temp = MLX_object();
		  float ambient = MLX_ambient();
		  char buf[100];
		  gcvt(temp, 8, buf);

		  if(temp > TEMP_THRESHOLD){
			  //We're pretty sure we're reading a forehead
			  //Show the temperature
			  OpenLCD_clear();
			  OpenLCD_setCursor(0, 0);
			  OpenLCD_writebuff(buf, 6);
			  if(temp < TEMP_SAFE){
				  //They're considered a safe temperature.
				  //Display it, and green
				  OpenLCD_setFastBacklightrgb(27, 240, 69);
				  state = PAUSE; //Give them some time to see it before clearing.
			  }else if(temp < TEMP_UNSAFE){
				  //Threshold temperature.
				  //Display it, and yellow, alongside instructions to touch contact sensor.
				  OpenLCD_setFastBacklightrgb(230, 255, 0);
				  OpenLCD_setCursor(0, 1);
				  OpenLCD_writestr(MESSAGE_TOUCH);
				  locked_temp = temp;
				  state = TOUCH_WAIT;
			  }else{
				  //Unsafe temperature.
				  //Display it, and red
				  OpenLCD_setFastBacklightrgb(240, 27, 105);
				  state = PAUSE;// Give them some time to see it before clearing.
			  }
		  }else{
			  printf("Detected temp is %f\r\n", temp);
			  OpenLCD_clear();
			  OpenLCD_setCursor(0, 0);
			  OpenLCD_writestr(MESSAGE_MOVE_CLOSER);
		  }

		  break;
	  case TOUCH_WAIT:
		  //The user isn't confirmed safe / unsafe; they're in-between
		  //Make them use the contact sensor for further evaluation
		  asm("nop");//Statement required after label.
		  float touch_temp = read_temperature_blocking();
		  char touch_buf[500];
		  gcvt(touch_temp, 8, touch_buf);
		  char locked_temp_buf[100];
		  gcvt(locked_temp, 8, locked_temp_buf);

		  OpenLCD_clear();
		  OpenLCD_setCursor(0, 0);
		  OpenLCD_writebuff(locked_temp_buf, 6);
		  if(touch_temp > TOUCH_TEMP_THRESHOLD){
			  counter = 0;
			  state = TOUCH_HOLD;
		  }else{
			  OpenLCD_setCursor(0, 1);
			  OpenLCD_writestr(MESSAGE_TOUCH);
			  counter++;
			  if(counter > TOUCH_TIMEOUT){
				 OpenLCD_clear();
				 OpenLCD_setFastBacklightrgb(128, 128, 128);
				 state = START;
			  }
		  }
		  break;
	  case TOUCH_HOLD:
		  counter++;
		  if(counter < TOUCH_HOLD_COUNT) break;
		  OpenLCD_setCursor(0, 1);
		  OpenLCD_writebuff(touch_buf, 6);
		  if(touch_temp < TOUCH_TEMP_SAFE){
		 		//They're considered a safe temperature.
		 		//Display it, and green
		 		OpenLCD_setFastBacklightrgb(27, 240, 69);
		 		counter = 0;
		 		state = PAUSE; //Give them some time to see it before clearing.
		  }else{
			  //Unsafe temperature.
			  //Display it, and red
			  OpenLCD_setFastBacklightrgb(240, 27, 105);
			  counter = 0;
			  state = PAUSE;// Give them some time to see it before clearing.
		  }
		  break;
	  case PAUSE:
		  counter++;
		  if(counter > DELAY_TIMEOUT){
			  OpenLCD_clear();
			  OpenLCD_setFastBacklightrgb(128, 128, 128);
			  state = START;
		  }
		  break;
	  }
	  HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35471;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
