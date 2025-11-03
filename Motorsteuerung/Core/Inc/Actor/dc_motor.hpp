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
private:
    MotorDriver& driver;
    Encoder& encoder;

    float currentSpeed;
    float currentPosition;

public:
    DCMotor(MotorDriver& driver, Encoder& encoder);

    void update(float dt);
    void setOutput(float value);

    float getSpeed() const;
    float getPosition() const;
};


#endif /* INC_ACTOR_DC_MOTOR_HPP_ */
