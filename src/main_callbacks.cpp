#include "main.hpp"
#include "main_prog.hpp"
#include "Timing.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
// #include "BNO055.hpp"
#include "BMP280.hpp"
#include "logger.hpp"
#include "can_messages.h"
#include "BNO055.hpp"

namespace se = stmepic;

void can_callback_bmp280_get_status(se::CanBase &can, se::CanDataFrame &msg, void *args) {

  auto bmp280 = static_cast<se::sensors::barometer::BMP280 *>(args);
  can_barometer_status_t status;
  auto stat = bmp280->device_get_status();
  if(stat.ok()) {
    status.status = CAN_BAROMETER_STATUS_STATUS_OK_CHOICE;
  } else {
    status.status = CAN_BAROMETER_STATUS_STATUS_ERROR_CHOICE;
  }
  se::CanDataFrame frame;
  frame.extended_id    = CAN_BAROMETER_STATUS_IS_EXTENDED;
  frame.frame_id       = CAN_BAROMETER_STATUS_FRAME_ID;
  frame.data_size      = CAN_BAROMETER_STATUS_LENGTH;
  frame.remote_request = false;
  can_barometer_status_pack(frame.data, &status, frame.data_size);
  (void)can.write(frame);
}

void can_callback_bmp280_get_data(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)args;
  auto bmp280 = static_cast<se::sensors::barometer::BMP280 *>(args);
  auto data   = bmp280->get_data();
  if(!data.ok()) {
    log_error(se::Logger::parse_to_json_format("BMP280", data.status().to_string(), false));
    return;
  }

  auto data_value = data.valueOrDie();
  can_barometer_data_t status;
  status.temperature = data_value.temp;
  status.pressure    = data_value.pressure;
  se::CanDataFrame frame;
  frame.extended_id    = CAN_BAROMETER_DATA_IS_EXTENDED;
  frame.frame_id       = CAN_BAROMETER_DATA_FRAME_ID;
  frame.data_size      = CAN_BAROMETER_DATA_LENGTH;
  frame.remote_request = false;
  can_barometer_data_pack(frame.data, &status, frame.data_size);
  (void)can.write(frame);
}

void can_callback_imu_status(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)args;
  auto imu = static_cast<se::sensors::imu::BNO055 *>(args);
  can_imu_status_t ms;
  auto imu_status = imu->device_get_status();
  if(imu_status.ok()) {
    ms.status = CAN_IMU_STATUS_STATUS_OK_CHOICE;
  } else {
    ms.status = CAN_IMU_STATUS_STATUS_ERROR_CHOICE;
  }
  se::CanDataFrame frame;
  frame.extended_id    = CAN_IMU_STATUS_IS_EXTENDED;
  frame.frame_id       = CAN_IMU_STATUS_FRAME_ID;
  frame.data_size      = CAN_IMU_STATUS_LENGTH;
  frame.remote_request = false;
  can_imu_status_pack(frame.data, &ms, frame.data_size);
  (void)can.write(frame);
}

void can_callback_imu_orientation(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)args;
  auto imu  = static_cast<se::sensors::imu::BNO055 *>(args);
  auto data = imu->get_data();
  if(!data.ok()) {
    // log_error(se::Logger::parse_to_json_format("IMU", data.status().to_string(), false));
    return;
  }
  auto data_value = data.valueOrDie();
  can_imu_orientation_t ms;
  ms.w = data_value.qua.w;
  ms.x = data_value.qua.x;
  ms.y = data_value.qua.y;
  ms.z = data_value.qua.z;

  se::CanDataFrame frame;
  frame.extended_id    = CAN_IMU_ORIENTATION_IS_EXTENDED;
  frame.frame_id       = CAN_IMU_ORIENTATION_FRAME_ID;
  frame.data_size      = CAN_IMU_ORIENTATION_LENGTH;
  frame.remote_request = false;
  can_imu_orientation_pack(frame.data, &ms, frame.data_size);
  (void)can.write(frame);
}

void can_callback_imu_lin_acceleration(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)args;
  auto imu  = static_cast<se::sensors::imu::BNO055 *>(args);
  auto data = imu->get_data();
  if(!data.ok()) {
    // log_error(se::Logger::parse_to_json_format("IMU", data.status().to_string(), false));
    return;
  }
  auto data_value = data.valueOrDie();
  can_imu_linear_acceleration_t ms;
  ms.x = data_value.lia.x;
  ms.y = data_value.lia.y;
  ms.z = data_value.lia.z;

  se::CanDataFrame frame;
  frame.extended_id    = CAN_IMU_LINEAR_ACCELERATION_IS_EXTENDED;
  frame.frame_id       = CAN_IMU_LINEAR_ACCELERATION_FRAME_ID;
  frame.data_size      = CAN_IMU_LINEAR_ACCELERATION_LENGTH;
  frame.remote_request = false;
  can_imu_linear_acceleration_pack(frame.data, &ms, frame.data_size);
  (void)can.write(frame);
}

void can_callback_imu_magnetic_field(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)args;
  auto imu  = static_cast<se::sensors::imu::BNO055 *>(args);
  auto data = imu->get_data();
  if(!data.ok()) {
    // log_error(se::Logger::parse_to_json_format("IMU", data.status().to_string(), false));
    return;
  }
  auto data_value = data.valueOrDie();
  can_imu_magnetic_field_t ms;
  ms.x = data_value.mag.x;
  ms.y = data_value.mag.y;
  ms.z = data_value.mag.z;

  se::CanDataFrame frame;
  frame.extended_id    = CAN_IMU_MAGNETIC_FIELD_IS_EXTENDED;
  frame.frame_id       = CAN_IMU_MAGNETIC_FIELD_FRAME_ID;
  frame.data_size      = CAN_IMU_MAGNETIC_FIELD_LENGTH;
  frame.remote_request = false;
  can_imu_magnetic_field_pack(frame.data, &ms, frame.data_size);
  (void)can.write(frame);
}


void can_callback_imu_gyration(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)args;
  auto imu  = static_cast<se::sensors::imu::BNO055 *>(args);
  auto data = imu->get_data();
  if(!data.ok()) {
    // log_error(se::Logger::parse_to_json_format("IMU", data.status().to_string(), false));
    return;
  }
  auto data_value = data.valueOrDie();
  can_imu_gyration_t ms;
  ms.x = data_value.gyr.x;
  ms.y = data_value.gyr.y;
  ms.z = data_value.gyr.z;

  se::CanDataFrame frame;
  frame.extended_id    = CAN_IMU_GYRATION_IS_EXTENDED;
  frame.frame_id       = CAN_IMU_GYRATION_FRAME_ID;
  frame.data_size      = CAN_IMU_GYRATION_LENGTH;
  frame.remote_request = false;
  can_imu_gyration_pack(frame.data, &ms, frame.data_size);
  (void)can.write(frame);
}