/*
 * PositionController.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */


#include <PositionController.hpp>

PositionController::PositionController(float kp, float ki, float kd)
    : PIDController(kp, ki, kd), targetPosition(0.0f), currentPosition(0.0f)
{}

void PositionController::setTargetPosition(float pos)
{
    targetPosition = pos;
}

float PositionController::getTargetPosition() const
{
    return targetPosition;
}

float PositionController::update(float measuredPosition, float dt)
{
    currentPosition = measuredPosition;
    // Das Ergebnis ist eine gewünschte Geschwindigkeit
    return compute(targetPosition, currentPosition, dt);
}

