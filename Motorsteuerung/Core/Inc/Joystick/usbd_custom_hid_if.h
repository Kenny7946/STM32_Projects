/*
 * usbd_custom_hid_if.hpp
 *
 *  Created on: Dec 12, 2025
 *      Author: marku
 */

#ifndef INC_JOYSTICK_USBD_CUSTOM_HID_IF_H_
#define INC_JOYSTICK_USBD_CUSTOM_HID_IF_H_

typedef struct {
    uint8_t axisX;    // Ein einzelner Achswert 0..255
} __packed HID_InputReport_t;

static HID_InputReport_t hidReport;



#endif /* INC_JOYSTICK_USBD_CUSTOM_HID_IF_H_ */
