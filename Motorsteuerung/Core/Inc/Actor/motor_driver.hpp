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
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef* dirPort;
    uint16_t dirPin;
    uint16_t pwmMax;

public:
    MotorDriver(TIM_HandleTypeDef* htim, uint32_t channel,
                GPIO_TypeDef* dirPort, uint16_t dirPin, uint16_t pwmMax = 999);

    void setOutput(float value);
};


#endif /* INC_ACTOR_MOTOR_DRIVER_HPP_ */
