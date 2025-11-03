/*
 * position_controller.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef CORE_INC_CONTROLLERTYPES_POSITION_CONTROLLER_HPP_
#define CORE_INC_CONTROLLERTYPES_POSITION_CONTROLLER_HPP_

extern "C" {
	#include "hal_include.hpp"
}
#include "pid_controller.hpp"


class PositionController : public PIDController {

public:
    PositionController(float kp, float ki, float kd);

    int32_t update(int32_t measured_position, float dt);


    int32_t target_position;
};



#endif /* CORE_INC_CONTROLLERTYPES_POSITION_CONTROLLER_HPP_ */
