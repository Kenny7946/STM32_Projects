/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmath>
#include <cstdint>
#include <string>
#include <cstring>

extern "C" {
#include "main.h"
#include "stm32l4xx_hal.h"
}

#include "dc_motor.hpp"
#include "motor_driver.hpp"
#include "encoder.hpp"
#include "pid_controller.hpp"
#include "position_controller.hpp"
#include "speed_controller.hpp"
#include <INA226.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 64
#define UART_RECEIVE_TIMEOUT 10
#define UART_SEND_TIMEOUT 50
#define ENCODER_RESOLUTION 8384   // Impulse pro Umdrehung
const uint8_t INA226_IC2_ADDRESS = (0x40 << 1);
const double SHUNT_RESISTOR_OHMS = 0.1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define SENSOR_ADDR 0x40
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//UART_HandleTypeDef huart2;
//I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

// -----------------------------------------
// Hardware-Setup
// -----------------------------------------
/*MotorDriver motorDriver(&htim1, TIM_CHANNEL_1, GPIOB, GPIO_PIN_5);
Encoder encoder(&htim2);
DCMotor dc_motor(motorDriver, encoder);

// -----------------------------------------
// PID-Regler
// -----------------------------------------
PositionController position_controller(0.0013,0.0,0.0);
SpeedController speed_controller(0.0,4.0,0.0);*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
//static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void UART_SEND(UART_HandleTypeDef *huart, char buffer[]);
void I2C_SEND(uint8_t port, uint8_t data);
float get_motor_speed();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t millis() {
    return HAL_GetTick();   // Gibt ms seit HAL_Init() zurück
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_TIM1_Init();
  //MX_TIM2_Init();
  //MX_USART2_UART_Init();
  //MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /*HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  const float dt = 0.01f; // Loopzeit in Sekunden (10 ms)
  float target_speed = 0.0f;

  position_controller.target_position = 0.0f; // Zielposition in Encoder-Ticks
  speed_controller.setTargetSpeed(target_speed);

  uint8_t rxBuffer[RX_BUFFER_SIZE];
  uint8_t rxIndex = 0;
  uint8_t ch;

  int32_t send_message_timer = 0;
  int32_t update_pid_controller_timer = 0;

  HAL_I2C_StateTypeDef test = HAL_I2C_GetState(&hi2c2);
  if(test == HAL_I2C_STATE_READY)
  {
	  __NOP();
  }
  else
  {
	  __NOP();
  }

  INA226 INA((0x40 << 1), &hi2c2);

  if(INA.begin())
  {
	  __NOP();
  }
  else
  {
	  __NOP();
  }

  float shunt = 0.118f;
  float current_LSB_mA = 2.5f;
  float current_zero_offset_mA = 0;
  uint16_t bus_V_scaling_e4 = 9872;

  int configure = INA.configure(shunt, current_LSB_mA, current_zero_offset_mA, bus_V_scaling_e4);
  if(configure)
  {
	  __NOP();
  }
  else
  {
	  __NOP();
  }


  HAL_Delay(10);


  // Globale oder statische Variablen für den gefilterten Wert
  static float filtered_voltage = 0.0f;
  static float filtered_current = 0.0f;

  // Filterkonstante (je kleiner, desto träger, z. B. 0.05 für starkes Filtern)
  const float alpha_voltage = 0.5f;
  const float alpha_current = 0.3f;*/


  uint8_t data;
  data = 1234;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(HAL_I2C_Mem_Read(&hi2c2, SENSOR_ADDR<<1, 0x0C, I2C_MEMADD_SIZE_8BIT, &data, 2, 1000) == HAL_OK){
		  __NOP();
	  } else {
		 __NOP();
	  }
	  HAL_Delay(100);

    /* USER CODE BEGIN 3 */
/*	  int32_t time = millis();
      if(time - update_pid_controller_timer >= 10)
      {
    	  int32_t currentPos = encoder.getCurrentValue();
    	  float motor_speed_rps = get_motor_speed();
    	  float time_diff = (float)(time - update_pid_controller_timer) / 1000.0f;
    	  update_pid_controller_timer = time;

    	  //float controlSignal = position_controller.update(currentPos, time_diff);
    	  float controlSignal = speed_controller.update(motor_speed_rps, time_diff);

    	  dc_motor.setOutput(controlSignal);

    	    // Messwerte vom INA226 lesen
    	    float voltage_raw = INA.getBusVoltage_mV() / 1000.0f;
    	    float current_raw = INA.getCurrent_mA();

    	    // Tiefpassfilter anwenden
    	    filtered_voltage = alpha_voltage * voltage_raw + (1.0f - alpha_voltage) * filtered_voltage;
    	    filtered_current = alpha_current * current_raw + (1.0f - alpha_current) * filtered_current;



     	 char msg[64];
     	//std::sprintf(msg, "%ld %ld %ld %f\r\n", time, (currentPos), position_controller.target_position, controlSignal);
     	 //std::sprintf(msg, "%ld %f %f %f\r\n", time, (motor_speed_rps), (speed_controller.getTargetSpeed()), (controlSignal));
     	//std::sprintf(msg, "%ld %.2fV %.1fmA\r\n", time, filtered_voltage, filtered_current);
     	std::sprintf(msg, "%.3fV %.3f\r\n", filtered_voltage, filtered_current);
 		 HAL_UART_Transmit(&huart2, (uint8_t*)msg, std::strlen(msg), UART_SEND_TIMEOUT);


      }





      if(time - send_message_timer > 40)
      {
    	  send_message_timer = time;
    	  //I2C_SEND(2u,3u);
      }

      if (HAL_UART_Receive(&huart2, &ch, 1, UART_RECEIVE_TIMEOUT) == HAL_OK) {
          if (ch == '\n' || ch == '\r') {
              rxBuffer[rxIndex] = '\0';
              int value = atoi((char*)rxBuffer);
              value *= 1;
              rxIndex = 0;
              //position_controller.target_position = value;
              speed_controller.setTargetSpeed((float)(value / 10.0f));
          } else if (rxIndex < RX_BUFFER_SIZE - 1) {
              rxBuffer[rxIndex++] = ch;
          }
      }*/

  }
  /* USER CODE END 3 */
}
/*
void UART_SEND(UART_HandleTypeDef *huart, char buffer[])
{
	HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), UART_SEND_TIMEOUT);
}

void I2C_SEND(uint8_t port, uint8_t data)
{
	const uint8_t size = 2;
	uint8_t buff[size];
	buff[0] = port;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c2, INA226_IC2_ADDRESS, buff, size, 1000);
}

float get_motor_speed() {
	static uint32_t last_count = 0;
	static uint32_t last_time = 0;
	static float velocity = 0.0f;
	static float motor_speed_rps_last_time = 0.0f;

	uint32_t now = millis();
	uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
	int32_t diff = (int32_t)(count - last_count);

	if (diff > 32768) diff -= 65536;
	if (diff < -32768) diff += 65536;

	int32_t motor_speed_ticks_per_second =  (float)(diff / ((now - last_time) / 1000.0f));
	float motor_speed_rps = (float)((float)motor_speed_ticks_per_second / ENCODER_RESOLUTION);

	// Tiefpassfilter
	velocity = 0.5f * velocity + 0.25f * motor_speed_rps + 0.25f * motor_speed_rps_last_time;


	motor_speed_rps_last_time = motor_speed_rps;
	last_count = count;
	last_time = now;

	return velocity;
}
*/
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
	/*
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  */
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  /*htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_MAX;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  //HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  /*htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  //hi2c1.Instance = I2C1;
  //hi2c1.Init.Timing = 0x00100D14;
  //hi2c1.Init.OwnAddress1 = 0;
  //hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  //hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  //hi2c1.Init.OwnAddress2 = 0;
  //hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  //hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  //hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  //if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  //{
    //Error_Handler();
  //}

  /** Configure Analogue filter
  */
  //if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  //{
    //Error_Handler();
  //}

  /** Configure Digital filter
  */
  //if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  //{
    //Error_Handler();
  //}
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00100D14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  __NOP();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
