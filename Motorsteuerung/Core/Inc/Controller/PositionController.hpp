/*
 * PositionController.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef INC_CONTROLLER_POSITIONCONTROLLER_HPP_
#define INC_CONTROLLER_POSITIONCONTROLLER_HPP_

#include <PIDController.hpp>
#include <PositionController.hpp>


class PositionController : public PIDController {
private:
    float targetPosition;
    float currentPosition;

public:
    PositionController(float kp, float ki, float kd);

    void setTargetPosition(float pos);
    float getTargetPosition() const;

    float update(float measuredPosition, float dt);
};



#endif /* INC_CONTROLLER_POSITIONCONTROLLER_HPP_ */
