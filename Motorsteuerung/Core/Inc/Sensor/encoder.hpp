/*
 * encoder.hpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#ifndef INC_SENSOR_ENCODER_HPP_
#define INC_SENSOR_ENCODER_HPP_

extern "C" {
	#include "hal_include.hpp"
}


class Encoder {
public:
    Encoder(TIM_HandleTypeDef* handle_timer);

    int32_t getCurrentValue() const;

private:
    TIM_HandleTypeDef* handle_timer;
};



#endif /* INC_SENSOR_ENCODER_HPP_ */
