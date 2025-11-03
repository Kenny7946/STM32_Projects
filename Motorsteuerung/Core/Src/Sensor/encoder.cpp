/*
 * encoder.cpp
 *
 *  Created on: Nov 3, 2025
 *      Author: marku
 */

#include <encoder.hpp>

Encoder::Encoder(TIM_HandleTypeDef* htim, uint16_t maxCount)
    : htim(htim), lastCount(0), totalCount(0), maxCount(maxCount)
{
    lastCount = __HAL_TIM_GET_COUNTER(htim);
}

void Encoder::update()
{
    int32_t count = __HAL_TIM_GET_COUNTER(htim);
    int32_t delta = count - lastCount;

    // Überlaufkorrektur
    if (delta > (maxCount / 2)) delta -= maxCount;
    else if (delta < -(maxCount / 2)) delta += maxCount;

    totalCount += delta;
    lastCount = count;
}

int32_t Encoder::getTotalCount() const
{
    return totalCount;
}

float Encoder::getSpeed(float dt)
{
    static int32_t lastTotal = 0;
    int32_t currentTotal = totalCount;
    int32_t delta = currentTotal - lastTotal;
    lastTotal = currentTotal;
    return delta / dt;
}



