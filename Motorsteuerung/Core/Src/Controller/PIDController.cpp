/*
 * PIDController.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include <PIDController.hpp>

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd), integral(0.0f), lastError(0.0f)
{}

float PIDController::compute(float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    integral += error * dt;
    float derivative = (error - lastError) / dt;
    lastError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}

void PIDController::reset()
{
    integral = 0.0f;
    lastError = 0.0f;
}


