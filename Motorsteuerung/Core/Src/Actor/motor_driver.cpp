/*
 * motor_driver.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include <motor_driver.hpp>
#include <cmath>

MotorDriver::MotorDriver(TIM_HandleTypeDef* handle_timer_, uint32_t channel_,
                         GPIO_TypeDef* dir_port_, uint16_t dir_pin_, uint16_t pwm_max_)
    : handle_timer(handle_timer_), channel(channel_), dir_port(dir_port_), dir_pin(dir_pin_), pwm_max(pwm_max_)
{}

void MotorDriver::setOutput(float value)
{
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    /*if(fabs(value) < 0.12f)
    {
    	value = 0.0f;
    }*/

    bool direction = (value >= 0.0f);
    uint16_t pwm_value = static_cast<uint16_t>(std::fabs(value) * pwm_max);

    HAL_GPIO_WritePin(dir_port, dir_pin, direction ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(handle_timer, channel, pwm_value);
}



