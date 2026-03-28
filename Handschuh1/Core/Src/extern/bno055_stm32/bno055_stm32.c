/*
 * bno055_stm32.c
 *
 *  Created on: Feb 26, 2026
 *      Author: marku
 */

#include "bno055_stm32.h"

I2C_HandleTypeDef *_bno055_i2c_port;

uint16_t bno055_i2c_address = BNO055_I2C_ADDR_LO;

void bno055_assignId(uint16_t id) {
	bno055_i2c_address = id;
}

void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device) {
	_bno055_i2c_port = hi2c_device;
}

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
	HAL_Delay(time);
#endif
}

static void bno055_select(bno055_dev_t *dev)
{
    bno055_assignI2C(dev->i2c);
    bno055_assignId(dev->address);
}

void bno055_init_dev(bno055_dev_t *dev)
{
    bno055_select(dev);
    bno055_setup();
    bno055_setOperationModeNDOF();
}

void bno055_getAllData(bno055_dev_t *dev, bno055_data_t *out)
{
    bno055_select(dev);

    //bno055_init_dev(dev);

    /*if (HAL_I2C_IsDeviceReady(_bno055_i2c_port, bno055_i2c_address << 1, 1, 50) == HAL_OK)
    {
    	__NOP();
    }
    else
    {
    	__NOP();
    }*/

    bno055_vector_t euler = bno055_getVectorEuler();
    bno055_vector_t acc   = bno055_getVectorAccelerometer();
    bno055_vector_t gyro  = bno055_getVectorGyroscope();
    bno055_vector_t mag   = bno055_getVectorMagnetometer();

    out->heading = euler.x;
    out->roll    = euler.y;
    out->pitch   = euler.z;

    out->acc_x = acc.x;
    out->acc_y = acc.y;
    out->acc_z = acc.z;

    out->gyro_x = gyro.x;
    out->gyro_y = gyro.y;
    out->gyro_z = gyro.z;

    out->mag_x = mag.x;
    out->mag_y = mag.y;
    out->mag_z = mag.z;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  HAL_I2C_Master_Transmit(_bno055_i2c_port, bno055_i2c_address << 1, &reg, 1,
                          100);
  HAL_I2C_Master_Receive(_bno055_i2c_port, bno055_i2c_address << 1, data, len,
                         100);
  // HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR_LO<<1, reg,
  // I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, bno055_i2c_address << 1,
                                   txdata, sizeof(txdata), 10);
  if (status == HAL_OK) {
    return;
  }

  if (status == HAL_ERROR) {
    printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
  } else if (status == HAL_TIMEOUT) {
    printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
  } else if (status == HAL_BUSY) {
    printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
  } else {
    printf("Unknown status data %d", status);
  }

  uint32_t error = HAL_I2C_GetError(_bno055_i2c_port);
  if (error == HAL_I2C_ERROR_NONE) {
    return;
  } else if (error == HAL_I2C_ERROR_BERR) {
    printf("HAL_I2C_ERROR_BERR\r\n");
  } else if (error == HAL_I2C_ERROR_ARLO) {
    printf("HAL_I2C_ERROR_ARLO\r\n");
  } else if (error == HAL_I2C_ERROR_AF) {
    printf("HAL_I2C_ERROR_AF\r\n");
  } else if (error == HAL_I2C_ERROR_OVR) {
    printf("HAL_I2C_ERROR_OVR\r\n");
  } else if (error == HAL_I2C_ERROR_DMA) {
    printf("HAL_I2C_ERROR_DMA\r\n");
  } else if (error == HAL_I2C_ERROR_TIMEOUT) {
    printf("HAL_I2C_ERROR_TIMEOUT\r\n");
  }

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(_bno055_i2c_port);
  if (state == HAL_I2C_STATE_RESET) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_READY) {
    printf("HAL_I2C_STATE_RESET\r\n");
  } else if (state == HAL_I2C_STATE_BUSY) {
    printf("HAL_I2C_STATE_BUSY\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX) {
    printf("HAL_I2C_STATE_BUSY_TX\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX) {
    printf("HAL_I2C_STATE_BUSY_RX\r\n");
  } else if (state == HAL_I2C_STATE_LISTEN) {
    printf("HAL_I2C_STATE_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
    printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
  } else if (state == HAL_I2C_STATE_ABORT) {
    printf("HAL_I2C_STATE_ABORT\r\n");
  }/* else if (state == HAL_I2C_STATE_TIMEOUT) {
    printf("HAL_I2C_STATE_TIMEOUT\r\n");
  } else if (state == HAL_I2C_STATE_ERROR) {
    printf("HAL_I2C_STATE_ERROR\r\n");
  }*/
  // while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
  // return;
}
