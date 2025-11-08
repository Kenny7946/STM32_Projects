/*
 * pid_controller.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef CORE_INC_CONTROLLERTYPES_PID_CONTROLLER_HPP_
#define CORE_INC_CONTROLLERTYPES_PID_CONTROLLER_HPP_

extern "C" {
	#include "hal_include.hpp"
}

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    virtual ~PIDController() = default;

    virtual float compute(float setpoint, float measurement, float dt);
    void reset();

protected:
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float lastError;
};



#endif /* CORE_INC_CONTROLLERTYPES_PID_CONTROLLER_HPP_ */
