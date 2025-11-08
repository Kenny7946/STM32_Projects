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

float PIDController::compute(float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    integral += (float)error * dt;
    float derivative = (float)(error - lastError) / dt;
    lastError = error;

    float p = Kp * error;
    float i = Ki * integral;
    float d = Kd * derivative;

    return p + i + d;
}

void PIDController::reset()
{
    integral = 0.0f;
    lastError = 0.0f;
}



