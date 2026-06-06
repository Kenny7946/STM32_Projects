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

void bno055_testStatus(bno055_dev_t *dev)
{
    uint8_t selftest_reg;
    uint8_t calib_reg;
    uint8_t opr_mode;

    /*
     * Register:
     * 0x35 -> Calibration Status
     * 0x36 -> Self Test Result
     * 0x3D -> Operation Mode
     */


    bno055_readData(0x35, &calib_reg, 1);
    bno055_readData(0x36, &selftest_reg, 1);
    bno055_readData(0x3D, &opr_mode, 1);


    /*
     * Decode selftest
     */

    uint8_t acc_test  = (selftest_reg >> 0) & 0x01;
    uint8_t mag_test  = (selftest_reg >> 1) & 0x01;
    uint8_t gyro_test = (selftest_reg >> 2) & 0x01;
    uint8_t mcu_test  = (selftest_reg >> 3) & 0x01;

    /*
     * Decode calibration
     */

    uint8_t sys_cal   = (calib_reg >> 6) & 0x03;
    uint8_t gyro_cal  = (calib_reg >> 4) & 0x03;
    uint8_t accel_cal = (calib_reg >> 2) & 0x03;
    uint8_t mag_cal   = (calib_reg >> 0) & 0x03;

    /*
     * Print raw values
     */

    printf("RAW_SELFTEST: 0x%02X\r\n", selftest_reg);
    printf("RAW_CALIB   : 0x%02X\r\n", calib_reg);
    printf("OPR_MODE    : 0x%02X\r\n", opr_mode);

    /*
     * Print decoded selftest
     */

    printf(
        "SELFTEST -> MCU:%u GYRO:%u MAG:%u ACC:%u\r\n",
        mcu_test,
        gyro_test,
        mag_test,
        acc_test
    );

    /*
     * Print decoded calibration
     */

    printf(
        "CALIB -> SYS:%u GYRO:%u MAG:%u ACC:%u\r\n",
        sys_cal,
        gyro_cal,
        mag_cal,
        accel_cal
    );

    /*
     * Helpful mode decode
     */

    switch(opr_mode)
    {
        case 0x00:
            printf("MODE: CONFIGMODE\r\n");
            break;

        case 0x08:
            printf("MODE: IMUPLUS\r\n");
            break;

        case 0x09:
            printf("MODE: COMPASS\r\n");
            break;

        case 0x0B:
            printf("MODE: NDOF_FMC_OFF\r\n");
            break;

        case 0x0C:
            printf("MODE: NDOF\r\n");
            break;

        default:
            printf("MODE: UNKNOWN\r\n");
            break;
    }

    printf("\r\n");
}

void myTask(void)
{
    uint16_t idx = 0;

    double heading1 = 0, heading2 = 0;

    HAL_Delay(10);

    bno055_data_t data;
    bno055_getAllData(&bno3, &data);

    printf(
        "ACC_X %lf ACC_Y %lf ACC_Z %lf\t"
        "GYRO_X %lf GYRO_Y %lf GYRO_Z %lf\t"
        "MAG_X %lf MAG_Y %lf MAG_Z %lf\t"
        "Heading: %lf\r\n",

        data.acc_x,
        data.acc_y,
        data.acc_z,

        data.gyro_x,
        data.gyro_y,
        data.gyro_z,

        data.mag_x,
        data.mag_y,
        data.mag_z,

        data.heading
    );


    //bno055_testStatus(&bno3);

    /*printf(
        "ADC0:%d ADC1:%d ADC2:%d ADC3:%d ADC4:%d\r\n",
        adc_values[0],
        adc_values[1],
        adc_values[2],
        adc_values[3],
        adc_values[4]
    );*/

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
    write_float_to_buffer(UpdateCharData, &idx, (float)linAccel.z);

    printf("EUL[%.2f %.2f %.2f] GRAV[%.2f %.2f %.2f] GYRO[%.2f %.2f %.2f] ACC[%.2f %.2f %.2f] MAG[%.2f %.2f %.2f] LIN[%.2f %.2f %.2f]\r\n",
    euler.x, euler.y, euler.z,
    gravity.x, gravity.y, gravity.z,
    gyro.x, gyro.y, gyro.z,
    accel.x, accel.y, accel.z,
    mag.x, mag.y, mag.z,
    linAccel.x, linAccel.y, linAccel.z);*/

    // --- BLE senden ---
    Custom_Writechar_Update_Char();
    Custom_Writechar_Send_Notification();

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
