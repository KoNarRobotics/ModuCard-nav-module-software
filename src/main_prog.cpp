#include "main.hpp"
#include "main_prog.hpp"
#include "Timing.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
#include "BNO055.hpp"
#include "BMP280.hpp"
#include "logger.hpp"
#include "can_messages.h"
#include "ICM20948.hpp"


std::shared_ptr<se::I2C> i2c1;
std::shared_ptr<se::sensors::imu::BNO055> bno055       = nullptr;
std::shared_ptr<se::sensors::barometer::BMP280> bmp280 = nullptr;
std::shared_ptr<se::sensors::imu::ICM20948> icm20948   = nullptr;
std::shared_ptr<se::FDCAN> fdcan                       = nullptr;

se::GpioPin gpio_i2c1_scl(*GPIOB, GPIO_PIN_8);
se::GpioPin gpio_i2c1_sda(*GPIOB, GPIO_PIN_9);
se::GpioPin gpio_boot_enable(*BOOT_EN_GPIO_Port, BOOT_EN_Pin);
se::GpioPin gpio_imu_nreset(*IMU_EXT_NRESET_GPIO_Port, IMU_EXT_NRESET_Pin);
se::GpioPin gpio_imu_interrupt(*IMU_INT_GPIO_Port, IMU_INT_Pin);

se::GpioPin gpio_user_led_1(*USER_LED_1_GPIO_Port, USER_LED_1_Pin);
se::GpioPin gpio_user_led_2(*USER_LED_2_GPIO_Port, USER_LED_2_Pin);
se::GpioPin gpio_status_led(*STATUS_LED_GPIO_Port, STATUS_LED_Pin);
se::GpioPin gpio_usr_button(*USR_BUTTON_GPIO_Port, USR_BUTTON_Pin);
se::GpioPin gpio_gps_geofence_stat(*GPS_GEOFENCE_STAT_GPIO_Port, GPS_GEOFENCE_STAT_Pin);
se::GpioPin gpio_gps_rtk_stat(*GPS_RTK_STAT_GPIO_Port, GPS_RTK_STAT_Pin);
se::GpioPin gpio_gps_exinterupt(*GPS_EXINTERUPT_GPIO_Port, GPS_EXINTERUPT_Pin);
se::GpioPin gpio_gps_mode_select(*GPS_MODE_SELECT_GPIO_Port, GPS_MODE_SELECT_Pin);
se::GpioPin gpio_gps_nreset(*GPS_NRESET_GPIO_Port, GPS_NRESET_Pin);


se::SimpleTask task_blink;


/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
extern "C" {
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM6) {
    se::Ticker::get_instance().irq_update_ticker();
  }

  if(htim->Instance == TIM7) {
    HAL_IncTick();
  }
}
}


void task_blink_func(se::SimpleTask &task, void *pvParameters) {
  // gpio_imu_nreset.write(1);
  gpio_user_led_1.write(0);
  gpio_user_led_2.write(0);
  se::Status stat = se::Status::OK();

  se::DeviceThreadedSettings settings;
  settings.uxStackDepth = 1024;
  settings.uxPriority   = 2;
  settings.period       = 10;
  se::sensors::imu::BNO0055_Settings bno055_settings;

  STMEPIC_ASSING_TO_OR_HRESET(bno055, se::sensors::imu::BNO055::Make(i2c1));
  bno055->device_set_settings(bno055_settings);
  bno055->device_task_set_settings(settings);
  STMEPIC_NONE_OR_HRESET(bno055->device_task_start());


  // STMEPIC_ASSING_TO_OR_HRESET(icm20948, se::sensors::imu::ICM20948::Make(i2c1,
  // se::sensors::imu::internal::ICM20948_I2C_ADDRESS_2)); icm20948->device_task_set_settings(settings);
  // STMEPIC_NONE_OR_HRESET(icm20948->device_task_start());


  STMEPIC_ASSING_TO_OR_HRESET(bmp280, se::sensors::barometer::BMP280::Make(i2c1));
  bmp280->device_task_set_settings(settings);
  // bmp280->device_start();
  STMEPIC_NONE_OR_HRESET(bmp280->device_task_start());

  fdcan->add_callback(CAN_BAROMETER_STATUS_FRAME_ID, can_callback_bmp280_get_status, bmp280.get());
  fdcan->add_callback(CAN_BAROMETER_DATA_FRAME_ID, can_callback_bmp280_get_data, bmp280.get());
  fdcan->add_callback(CAN_IMU_STATUS_FRAME_ID, can_callback_imu_status, bno055.get());
  fdcan->add_callback(CAN_IMU_ORIENTATION_FRAME_ID, can_callback_imu_orientation, bno055.get());
  fdcan->add_callback(CAN_IMU_LINEAR_ACCELERATION_FRAME_ID, can_callback_imu_lin_acceleration, bno055.get());
  fdcan->add_callback(CAN_IMU_MAGNETIC_FIELD_FRAME_ID, can_callback_imu_magnetic_field, bno055.get());
  fdcan->add_callback(CAN_IMU_GYRATION_FRAME_ID, can_callback_imu_gyration, bno055.get());

  while(1) {
    vTaskDelay(100);

    // auto a = bno055->get_data();
    // if(a.ok()) {
    auto d = bno055->get_calibration_data();
    if(d.calibrated) {
      gpio_user_led_2.toggle();
      log_debug("IMU calibrated");
    } else {
      gpio_user_led_2.write(0);
      log_debug("IMU not calibrated");
    }
    // auto data = a.valueOrDie();
    // log_debug("IMU acc: " + std::to_string(data.acc.x) + " " + std::to_string(data.acc.y) + " " +
    //           std::to_string(data.acc.z) + "IMU gyr: " + std::to_string(data.gyr.x) + " " +
    //           std::to_string(data.gyr.y) + " " + std::to_string(data.gyr.z) + "IMU mag: " +
    //           std::to_string(data.mag.x) + " " + std::to_string(data.mag.y) + " " + std::to_string(data.mag.z) +
    //           "IMU temp: " + std::to_string(data.temp) + "  Cal:" + std::to_string(d.calibrated));
    // } else {
    //   log_error("IMU error: " + a.status().to_string());
    // }

    gpio_status_led.toggle();
  }
}


void can_callback(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)can;
  (void)args;
  // log_debug(msg.to_string());
}


void main_prog() {
  HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);
  // MX_USB_PCD_Init();

  // MX_USB_DEVICE_Init();

  std::string version =
  std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) + "." + std::to_string(VERSION_BUILD);
  // CDC_Transmit_FS
  se::Logger::get_instance().init(se::LOG_LEVEL::LOG_LEVEL_DEBUG, true, nullptr, true, version);


  STMEPIC_ASSING_TO_OR_HRESET(i2c1, se::I2C::Make(hi2c1, gpio_i2c1_sda, gpio_i2c1_scl, se::HardwareType::DMA));
  i2c1->hardware_reset();


  task_blink.task_init(task_blink_func, nullptr, 100, nullptr, 2500);
  task_blink.task_run();

  FDCAN_FilterTypeDef sFilterConfig = {};
  sFilterConfig.IdType              = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex         = 0;
  sFilterConfig.FilterType          = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig        = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1           = 0; // 0x915;
  sFilterConfig.FilterID2           = 0; // 0x1FFFFFFF; // all have to match
  // sFilterConfig.RxBufferIndex       = 0;
  // sFilterConfig.IsCalibrationMsg    = 0;

  se::FDcanFilterConfig filter_config;
  filter_config.filters.push_back(sFilterConfig);
  filter_config.fifo_number                  = se::FDCAN_FIFO::FDCAN_FIFO0;
  filter_config.globalFilter_NonMatchingStd  = FDCAN_REJECT;
  filter_config.globalFilter_NonMatchingExt  = FDCAN_REJECT;
  filter_config.globalFilter_RejectRemoteStd = FDCAN_FILTER_REMOTE;
  filter_config.globalFilter_RejectRemoteExt = FDCAN_FILTER_REMOTE;


  STMEPIC_ASSING_TO_OR_HRESET(fdcan, se::FDCAN::Make(hfdcan1, filter_config, nullptr, nullptr));
  fdcan->hardware_start();
}