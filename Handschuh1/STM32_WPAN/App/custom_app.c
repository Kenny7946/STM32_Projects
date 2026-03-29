/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_stm32/bno055_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* BlService */
  uint8_t               Writechar_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
extern volatile uint16_t adc_values[2];
extern bno055_dev_t bno1, bno2, bno3, bno4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* BlService */
static void Custom_Writechar_Update_Char(void);
static void Custom_Writechar_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void write_float_to_buffer(uint8_t *buffer, uint16_t *index, float value)
{
    memcpy(&buffer[*index], &value, sizeof(float));
    *index += sizeof(float);
}

void myTask(void)
{
	/*static uint16_t index = 0;
	UpdateCharData[0] = index++;
	Custom_Writechar_Update_Char();
	UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);
	return;*/

    uint16_t idx = 0;

    double heading1 = 0, heading2 = 0;

    bno055_data_t data;
    bno055_getAllData(&bno4, &data);
    heading1 = data.heading;
    //bno055_getAllData(&bno2, &data);
    heading2 = data.heading;
    printf("Heading1: %lf\tHeading2: %lf\r\n",heading1, heading2);
    //bno055_getAllData(&bno2, &data);
    //bno055_getAllData(&bno3, &data);
    //bno055_getAllData(&bno4, &data);





    // --- ADC Werte ---
    /*UpdateCharData[idx++] = (uint8_t)((adc_values[0] >> 8) & 0xFF);
    UpdateCharData[idx++] = (uint8_t)(adc_values[0] & 0xFF);

    UpdateCharData[idx++] = (uint8_t)((adc_values[1] >> 8) & 0xFF);
    UpdateCharData[idx++] = (uint8_t)(adc_values[1] & 0xFF);

    UpdateCharData[idx++] = (uint8_t)((adc_values[2] >> 8) & 0xFF);
    UpdateCharData[idx++] = (uint8_t)(adc_values[2] & 0xFF);

    UpdateCharData[idx++] = (uint8_t)((adc_values[3] >> 8) & 0xFF);
    UpdateCharData[idx++] = (uint8_t)(adc_values[3] & 0xFF);

    UpdateCharData[idx++] = (uint8_t)((adc_values[4] >> 8) & 0xFF);
    UpdateCharData[idx++] = (uint8_t)(adc_values[4] & 0xFF);

    // --- BNO055 Vektoren holen ---
    bno055_vector_t euler      = bno055_getVectorEuler();
    bno055_vector_t gravity    = bno055_getVectorGravity();
    bno055_vector_t gyro       = bno055_getVectorGyroscope();
    bno055_vector_t accel      = bno055_getVectorAccelerometer();
    bno055_vector_t mag        = bno055_getVectorMagnetometer();
    bno055_vector_t linAccel   = bno055_getVectorLinearAccel();



    // --- Alle Werte als float in Buffer schreiben ---
    write_float_to_buffer(UpdateCharData, &idx, (float)euler.x);
    write_float_to_buffer(UpdateCharData, &idx, (float)euler.y);
    write_float_to_buffer(UpdateCharData, &idx, (float)euler.z);

    write_float_to_buffer(UpdateCharData, &idx, (float)gravity.x);
    write_float_to_buffer(UpdateCharData, &idx, (float)gravity.y);
    write_float_to_buffer(UpdateCharData, &idx, (float)gravity.z);

    write_float_to_buffer(UpdateCharData, &idx, (float)gyro.x);
    write_float_to_buffer(UpdateCharData, &idx, (float)gyro.y);
    write_float_to_buffer(UpdateCharData, &idx, (float)gyro.z);

    write_float_to_buffer(UpdateCharData, &idx, (float)accel.x);
    write_float_to_buffer(UpdateCharData, &idx, (float)accel.y);
    write_float_to_buffer(UpdateCharData, &idx, (float)accel.z);

    write_float_to_buffer(UpdateCharData, &idx, (float)mag.x);
    write_float_to_buffer(UpdateCharData, &idx, (float)mag.y);
    write_float_to_buffer(UpdateCharData, &idx, (float)mag.z);

    write_float_to_buffer(UpdateCharData, &idx, (float)linAccel.x);
    write_float_to_buffer(UpdateCharData, &idx, (float)linAccel.y);
    write_float_to_buffer(UpdateCharData, &idx, (float)linAccel.z);*/

    // --- BLE senden ---
    Custom_Writechar_Update_Char();
    //Custom_Writechar_Send_Notification();

    UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK, CFG_SCH_PRIO_0);
}
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* BlService */
    case CUSTOM_STM_CHARWRITE_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CHARWRITE_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_CHARWRITE_WRITE_EVT */
      break;

    case CUSTOM_STM_WRITECHAR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_WRITECHAR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_WRITECHAR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_WRITECHAR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_WRITECHAR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_WRITECHAR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* BlService */
__USED void Custom_Writechar_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Writechar_UC_1*/
  updateflag = 1u;
  /* USER CODE END Writechar_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_WRITECHAR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Writechar_UC_Last*/

  /* USER CODE END Writechar_UC_Last*/
  return;
}

void Custom_Writechar_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Writechar_NS_1*/
  updateflag = 1u;
  /* USER CODE END Writechar_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_WRITECHAR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Writechar_NS_Last*/

  /* USER CODE END Writechar_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
