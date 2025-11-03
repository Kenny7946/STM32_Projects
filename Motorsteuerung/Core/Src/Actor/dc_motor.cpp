/*
 * dc_motor.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */


#include "dc_motor.hpp"

DCMotor::DCMotor(MotorDriver& driver, Encoder& encoder)
    : driver(driver), encoder(encoder)
{}

void DCMotor::setOutput(float value)
{
    driver.setOutput(value);
}

int32_t DCMotor::getSpeed() const { return 0; }
int32_t DCMotor::getPosition() const { return encoder.getCurrentValue(); }


