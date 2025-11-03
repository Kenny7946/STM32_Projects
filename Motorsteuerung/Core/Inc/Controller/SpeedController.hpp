/*
 * SpeedController.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef INC_CONTROLLER_SPEEDCONTROLLER_HPP_
#define INC_CONTROLLER_SPEEDCONTROLLER_HPP_

#include <PIDController.hpp>
#include <SpeedController.hpp>

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



#endif /* INC_CONTROLLER_SPEEDCONTROLLER_HPP_ */
