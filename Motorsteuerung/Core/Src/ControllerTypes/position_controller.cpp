/*
 * position_controller.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include "position_controller.hpp"

PositionController::PositionController(float kp, float ki, float kd)
    : PIDController(kp, ki, kd)
{}

int32_t PositionController::update(int32_t measured_position, float dt)
{
    return compute((float) target_position, (float)measured_position, dt);
}


