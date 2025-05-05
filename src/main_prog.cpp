#include "main.hpp"
#include "main_prog.hpp"
#include "Timing.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"
// #include "BNO055.hpp"
#include "BMP280.hpp"
#include "logger.hpp"
#include "can_messages.h"


std::shared_ptr<se::I2C> i2c1;
// std::shared_ptr<se::sensors::imu::BNO055> bno055       = nullptr;
std::shared_ptr<se::sensors::barometer::BMP280> bmp280 = nullptr;
std::shared_ptr<se::FDCAN> fdcan                       = nullptr;

se::GpioPin gpio_i2c1_scl(*GPIOB, GPIO_PIN_6);
se::GpioPin gpio_i2c1_sda(*GPIOB, GPIO_PIN_9);
se::GpioPin gpio_boot_enable(*BOOT_EN_GPIO_Port, BOOT_EN_Pin);
se::GpioPin gpio_imu_nreset(*IMU_NRESET_GPIO_Port, IMU_NRESET_Pin);
se::GpioPin gpio_imu_exinterupt(*IMU_EXINTERUPT_GPIO_Port, IMU_EXINTERUPT_Pin);

se::GpioPin gpio_user_led_1(*USER_LED_1_GPIO_Port, USER_LED_1_Pin);
se::GpioPin gpio_user_led_2(*USER_LED_2_GPIO_Port, USER_LED_2_Pin);
se::GpioPin gpio_status_led(*STATUS_LED_GPIO_Port, STATUS_LED_Pin);
se::GpioPin gpio_usr_button(*USR_BUTTON_GPIO_Port, USR_BUTTON_Pin);
se::GpioPin gpio_gps_geofence_stat(*GPS_GEOFENCE_STAT_GPIO_Port, GPS_GEOFENCE_STAT_Pin);
se::GpioPin gpio_gps_rtk_stat(*GPS_RTK_STAT_GPIO_Port, GPS_RTK_STAT_Pin);
se::GpioPin gpio_gps_exinterupt(*GPS_EXINTERUPT_GPIO_Port, GPS_EXINTERUPT_Pin);
se::GpioPin gpio_gps_mode_select(*GPS_MODE_SELECT_GPIO_Port, GPS_MODE_SELECT_Pin);
se::GpioPin gpio_gps_nreset(*GPS_NRESET_GPIO_Port, GPS_NRESET_Pin);

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

se::SimpleTask task_blink;

void task_blink_func(se::SimpleTask &task, void *pvParameters) {
  gpio_imu_nreset.write(1);
  gpio_user_led_1.write(0);
  gpio_user_led_2.write(0);
  se::Status stat = se::Status::OK();

  se::DeviceThreadedSettings settings;
  settings.uxStackDepth = 512;
  settings.uxPriority   = 2;
  settings.period       = 10;

  STMEPIC_ASSING_TO_OR_HRESET(bmp280, se::sensors::barometer::BMP280::Make(i2c1));
  bmp280->device_task_set_settings(settings);
  bmp280->device_start();
  STMEPIC_NONE_OR_HRESET(bmp280->device_task_start());

  fdcan->add_callback(CAN_BAROMETER_STATUS_FRAME_ID, can_callback_bmp280_get_status, bmp280.get());
  fdcan->add_callback(CAN_BAROMETER_DATA_FRAME_ID, can_callback_bmp280_get_data, bmp280.get());

  se::CanDataFrame frame;
  frame.extended_id    = true;
  frame.frame_id       = 0x123;
  frame.data[0]        = 0;
  frame.data_size      = 1;
  frame.remote_request = false;
  while(1) {
    vTaskDelay(100);
    gpio_user_led_1.toggle();
  }
}


void can_callback(se::CanBase &can, se::CanDataFrame &msg, void *args) {
  (void)can;
  (void)args;
  // log_debug(msg.to_string());
}


void main_prog() {
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);

  se::Logger::get_instance().init(se::LOG_LEVEL::LOG_LEVEL_DEBUG, true, nullptr, true, "0.0.1");


  STMEPIC_ASSING_TO_OR_HRESET(i2c1, se::I2C::Make(hi2c1, gpio_i2c1_sda, gpio_i2c1_scl, se::HardwareType::IT));
  i2c1->hardware_reset();


  // STMEPIC_ASSING_TO_OR_HRESET(bno055, se::sensors::imu::BNO055::Make(i2c1, nullptr, nullptr));
  // // bno055->device_task_set_settings(settings);
  // STMEPIC_NONE_OR_HRESET(bno055->device_task_start());

  task_blink.task_init(task_blink_func, nullptr, 100, nullptr, 600);
  task_blink.task_run();

  FDCAN_FilterTypeDef sFilterConfig = {};
  sFilterConfig.IdType              = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex         = 0;
  sFilterConfig.FilterType          = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig        = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1           = 0; // 0x915;
  sFilterConfig.FilterID2           = 0; // 0x1FFFFFFF; // all have to match
  sFilterConfig.RxBufferIndex       = 0;
  sFilterConfig.IsCalibrationMsg    = 0;

  se::FDcanFilterConfig filter_config;
  filter_config.filters.push_back(sFilterConfig);
  filter_config.fifo_number                  = se::FDCAN_FIFO::FDCAN_FIFO0;
  filter_config.globalFilter_NonMatchingStd  = FDCAN_REJECT;
  filter_config.globalFilter_NonMatchingExt  = FDCAN_REJECT;
  filter_config.globalFilter_RejectRemoteStd = FDCAN_FILTER_REMOTE;
  filter_config.globalFilter_RejectRemoteExt = FDCAN_FILTER_REMOTE;


  STMEPIC_ASSING_TO_OR_HRESET(fdcan, se::FDCAN::Make(hfdcan1, filter_config, nullptr, nullptr));
  fdcan->hardware_start();

  // se::Ticker::get_instance().init(&htim6);

  // Your code here like your tasks, drivers, etc.
  // Do not start FreeRTOS kernel here since it will be start later in main.cpp
}