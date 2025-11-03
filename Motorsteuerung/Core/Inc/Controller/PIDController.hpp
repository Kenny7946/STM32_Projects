/*
 * PIDController.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef INC_CONTROLLER_PIDCONTROLLER_HPP_
#define INC_CONTROLLER_PIDCONTROLLER_HPP_


class PIDController {
protected:
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float lastError;

public:
    PIDController(float kp, float ki, float kd);
    virtual ~PIDController() = default;

    virtual float compute(float setpoint, float measurement, float dt);
    void reset();
};

#endif /* INC_CONTROLLER_PIDCONTROLLER_HPP_ */
