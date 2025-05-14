#pragma once
#include "main.hpp"
#include "version.hpp"
#include "stmepic.hpp"
#include "fdcan.hpp"


// USB Device includes
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_composite_builder.h"
#include "usb_device.h"


namespace se = stmepic;

void config_usb_device();

void main_prog();

void can_callback_bmp280_get_status(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_bmp280_get_data(se::CanBase &can, se::CanDataFrame &msg, void *args);

void can_callback_imu_status(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_orientation(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_lin_acceleration(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_magnetic_field(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_gyration(se::CanBase &can, se::CanDataFrame &msg, void *args);