/*
 * pid_controller.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include "pid_controller.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : Kp(kp), Ki(ki), Kd(kd), integral(0.0f), lastError(0.0f)
{}

float PIDController::compute(int32_t setpoint, int32_t measurement, float dt)
{
    int32_t error = setpoint - measurement;

    integral += (float)error * dt;
    float derivative = float(error - lastError) / dt;
    lastError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}

void PIDController::reset()
{
    integral = 0.0f;
    lastError = 0.0f;
}



