#pragma once
#include "main.hpp"
#include "version.hpp"
#include "stmepic.hpp"
#include "fdcan.hpp"

namespace se = stmepic;

void main_prog();

void can_callback_bmp280_get_status(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_bmp280_get_data(se::CanBase &can, se::CanDataFrame &msg, void *args);

void can_callback_imu_status(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_orientation(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_lin_acceleration(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_magnetic_field(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_imu_gyration(se::CanBase &can, se::CanDataFrame &msg, void *args);