/*
 * dc_motor.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef INC_ACTOR_DC_MOTOR_HPP_
#define INC_ACTOR_DC_MOTOR_HPP_


#include "encoder.hpp"
#include "motor_driver.hpp"

class DCMotor {
public:
    DCMotor(MotorDriver& driver, Encoder& encoder);

    void setOutput(float value);

    int32_t getSpeed() const;
    int32_t getPosition() const;

private:
    MotorDriver& driver;
    Encoder& encoder;
};


#endif /* INC_ACTOR_DC_MOTOR_HPP_ */
