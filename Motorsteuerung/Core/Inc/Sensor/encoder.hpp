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
private:
    TIM_HandleTypeDef* htim;
    int32_t lastCount;
    int32_t totalCount;
    uint16_t maxCount;

public:
    Encoder(TIM_HandleTypeDef* htim, uint16_t maxCount = 65535);

    void update();
    int32_t getTotalCount() const;
    float getSpeed(float dt);
};



#endif /* INC_SENSOR_ENCODER_HPP_ */
