/*
 * speed_controller.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef CORE_INC_CONTROLLERTYPES_SPEED_CONTROLLER_HPP_
#define CORE_INC_CONTROLLERTYPES_SPEED_CONTROLLER_HPP_

#include "pid_controller.hpp"

class SpeedController : public PIDController {
private:
    float targetSpeed;
    float currentSpeed;

public:
    SpeedController(float kp, float ki, float kd);

    void setTargetSpeed(float speed);
    float getTargetSpeed() const;

    float update(float measuredSpeed, float dt);
};


#endif /* CORE_INC_CONTROLLERTYPES_SPEED_CONTROLLER_HPP_ */
