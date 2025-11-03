/*
 * motor_driver.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include <motor_driver.hpp>
#include <cmath>

MotorDriver::MotorDriver(TIM_HandleTypeDef* htim, uint32_t channel,
                         GPIO_TypeDef* dirPort, uint16_t dirPin, uint16_t pwmMax)
    : htim(htim), channel(channel), dirPort(dirPort), dirPin(dirPin), pwmMax(pwmMax)
{}

void MotorDriver::setOutput(float value)
{
    // Begrenzen
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    bool direction = (value >= 0.0f);
    uint16_t pwmValue = static_cast<uint16_t>(std::fabs(value) * pwmMax);

    HAL_GPIO_WritePin(dirPort, dirPin, direction ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(htim, channel, pwmValue);
}



