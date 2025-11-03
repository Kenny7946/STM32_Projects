/*
 * encoder.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include <encoder.hpp>

Encoder::Encoder(TIM_HandleTypeDef* handle_timer_)
    : handle_timer(handle_timer_)
{
}

int32_t Encoder::getCurrentValue() const
{
	return (int32_t) __HAL_TIM_GET_COUNTER(handle_timer);
}


