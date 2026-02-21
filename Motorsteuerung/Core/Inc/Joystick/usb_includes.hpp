/*
 * usb_includes.hpp
 *
 *  Created on: Dec 12, 2025
 *      Author: marku
 */

#ifndef INC_JOYSTICK_USB_INCLUDES_HPP_
#define INC_JOYSTICK_USB_INCLUDES_HPP_

extern "C" {
    #include "usbd_custom_hid_if.h"
    #include "usbd_core.h"
    #include "usbd_def.h"
	#include "usb_device.h"
}

extern "C" {
    extern USBD_HandleTypeDef hUsbDeviceFS;
}



#endif /* INC_JOYSTICK_USB_INCLUDES_HPP_ */
