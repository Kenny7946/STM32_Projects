/*
 * SpeedController.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include <SpeedController.hpp>

SpeedController::SpeedController(float kp, float ki, float kd)
    : PIDController(kp, ki, kd), targetSpeed(0.0f), currentSpeed(0.0f)
{}

void SpeedController::setTargetSpeed(float speed)
{
    targetSpeed = speed;
}

float SpeedController::getTargetSpeed() const
{
    return targetSpeed;
}

float SpeedController::update(float measuredSpeed, float dt)
{
    currentSpeed = measuredSpeed;
    return compute(targetSpeed, currentSpeed, dt);
}



