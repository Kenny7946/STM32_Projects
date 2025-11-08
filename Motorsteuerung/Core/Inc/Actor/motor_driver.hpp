/*
 * motor_driver.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef INC_ACTOR_MOTOR_DRIVER_HPP_
#define INC_ACTOR_MOTOR_DRIVER_HPP_

extern "C" {
	#include "hal_include.hpp"
}



class MotorDriver {
private:
    TIM_HandleTypeDef* handle_timer;
    uint32_t channel;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    uint16_t pwm_max;

public:
    MotorDriver(TIM_HandleTypeDef* handle_timer, uint32_t channel,
                GPIO_TypeDef* dir_port, uint16_t dir_pin, uint16_t pwm_max = PWM_MAX);

    void setOutput(float value);
};


#endif /* INC_ACTOR_MOTOR_DRIVER_HPP_ */
