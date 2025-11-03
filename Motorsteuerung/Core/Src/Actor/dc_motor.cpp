/*
 * dc_motor.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */


#include "dc_motor.hpp"

DCMotor::DCMotor(MotorDriver& driver, Encoder& encoder)
    : driver(driver), encoder(encoder), currentSpeed(0), currentPosition(0)
{}

void DCMotor::update(float dt)
{
    encoder.update();
    currentSpeed = encoder.getSpeed(dt);
    currentPosition = static_cast<float>(encoder.getTotalCount());
}

void DCMotor::setOutput(float value)
{
    driver.setOutput(value);
}

float DCMotor::getSpeed() const { return currentSpeed; }
float DCMotor::getPosition() const { return currentPosition; }


