#include "main.hpp"
#include "main_prog.hpp"
#include "Timing.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
// #include "BNO055.hpp"
#include "BMP280.hpp"
#include "logger.hpp"
#include "can_messages.h"

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