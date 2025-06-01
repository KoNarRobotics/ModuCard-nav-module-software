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
#include "usbd_cdc_if.h"
#include "uart.hpp"
#include "atmodem.hpp"


std::shared_ptr<se::I2C> i2c1;
std::shared_ptr<se::sensors::imu::BNO055> bno055       = nullptr;
std::shared_ptr<se::sensors::barometer::BMP280> bmp280 = nullptr;
std::shared_ptr<se::sensors::imu::ICM20948> icm20948   = nullptr;
std::shared_ptr<se::modems::AtModem> atmodem           = nullptr;

std::shared_ptr<se::UART> uart4  = nullptr;
std::shared_ptr<se::FDCAN> fdcan = nullptr;

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
    HAL_IncTick();
  }

  if(htim->Instance == TIM7) {
    HAL_IncTick();
  }
}
}


char rx_buff[512];

// extern "C" {
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hi2c) {
//   HAL_UART_Receive_IT(&huart4, (uint8_t *)rx_buff, sizeof(rx_buff));
//   memset(rx_buff, 0, sizeof(rx_buff));
// }

// void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *hi2c) {
//   // UART::run_rx_callbacks_from_isr(hi2c, true);
//   __NOP();
// }
// }

void task_blink_func(se::SimpleTask &task, void *pvParameters) {
  // gpio_imu_nreset.write(1);
  gpio_user_led_1.write(0);
  gpio_user_led_2.write(0);
  se::Status stat = se::Status::OK();

  se::modems::AtModemSettings atmodem_settings;
  atmodem_settings.enable_gps = true; // Enable GPS by default

  se::DeviceThreadedSettings settings;
  settings.uxStackDepth = 1024;
  settings.uxPriority   = 2;
  settings.period       = 10;
  se::sensors::imu::BNO0055_Settings bno055_settings;


  se::DeviceThreadedSettings settings_at;
  settings.uxStackDepth = 4024;
  settings.uxPriority   = 2;
  settings.period       = 10;
  STMEPIC_ASSING_TO_OR_HRESET(atmodem, se::modems::AtModem::Make(uart4));
  atmodem->device_set_settings(atmodem_settings);
  atmodem->device_task_set_settings(settings);
  // atmodem->device_reset();
  STMEPIC_NONE_OR_HRESET(atmodem->device_task_start());


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

  fdcan->add_callback(CAN_GPS_STATUS_FRAME_ID, can_callback_gps_status, atmodem.get());
  fdcan->add_callback(CAN_GPS_LATITUDE_FRAME_ID, can_callback_gps_latitude, atmodem.get());
  fdcan->add_callback(CAN_GPS_LONGITUDE_FRAME_ID, can_callback_gps_longitude, atmodem.get());
  fdcan->add_callback(CAN_GPS_ALTITUDE_FRAME_ID, can_callback_gps_altitude, atmodem.get());
  fdcan->add_callback(CAN_GPS_DATE_FRAME_ID, can_callback_gps_date, atmodem.get());
  fdcan->add_callback(CAN_GPS_COVARIANCE_FRAME_ID, can_callback_gps_covariance, atmodem.get());

  bno055->device_reset();

  // HAL_StatusTypeDef s;
  // s = HAL_UART_Receive_IT(&huart4, (uint8_t *)rx_buff, sizeof(rx_buff));
  while(1) {
    vTaskDelay(100);

    // auto a = bno055->get_data();
    // if(a.ok()) {
    // auto d = bno055->get_calibration_data();
    // char *str = "AT\r\n";
    // // s         = HAL_UART_Transmit_IT(&huart4, (uint8_t *)str, sizeof(str));
    // auto a       = uart4->write((uint8_t *)str, sizeof(str), 100);
    // char data[8] = { 0 };
    // uart4->read((uint8_t *)data, sizeof(data), 2000);


    // auto a = atmodem->get_nmea_data();
    // if(a.ok()) {
    //   log_info("NMEA data received successfully");
    //   // auto &nmea_data = a.valueOrDie();
    //   // auto gga        = nmea_data.get_gga_data();
    //   // log_info("NMEA data: " + std::to_string(gga.longitude) + " " + std::to_string(gga.latitude) + " " +
    //   //          std::to_string(gga.altitude) + " " + std::to_string(gga.fix_quality) + " " +
    //   //          std::to_string(gga.num_satellites));
    // } else {
    //   log_error("NMEA data error: " + a.status().to_string());
    // }

    // auto d = bno055->get_calibration_data();
    // if(d.calibrated) {
    //   gpio_user_led_2.toggle();
    //   log_debug("IMU calibrated");
    // } else {
    //   gpio_user_led_2.write(0);
    //   log_debug("IMU not calibrated");
    // }
    // auto data = a.valueOrDie();
    // log_debug("IMU acc: " + std::to_string(data.acc.x) + " " + std::to_string(data.acc.y) + " " +
    //           std::to_string(data.acc.z) + "IMU gyr: " + std::to_string(data.gyr.x) + " " +
    //           std::to_string(data.gyr.y) + " " + std::to_string(data.gyr.z) + "IMU mag: " +
    //           std::to_string(data.mag.x) + " " + std::to_string(data.mag.y) + " " + std::to_string(data.mag.z) +
    //           "IMU temp: " + std::to_string(data.temp) + "  Cal:" + std::to_string(d.calibrated));

    gpio_status_led.toggle();
  }
}


void main_prog() {
  HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);
  // MX_USB_PCD_Init();

  // MX_USB_Device_Init();
  MX_USB_PCD_Init();
  MX_USB_Device_Init();

  std::string version =
  std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) + "." + std::to_string(VERSION_BUILD);
  // CDC_Transmit_FS TEMPLATE_Transmit
  se::Logger::get_instance().init(se::LOG_LEVEL::LOG_LEVEL_DEBUG, true, TEMPLATE_Transmit, false, version);


  STMEPIC_ASSING_TO_OR_HRESET(uart4, se::UART::Make(huart4, se::HardwareType::IT));
  uart4->hardware_start();
  // HAL_StatusTypeDef s;
  // s = HAL_UART_Receive_IT(&huart4, (uint8_t *)rx_buff, sizeof(rx_buff));
  // while(1) {
  //   char *str = "AT\r\n";
  //   s         = HAL_UART_Transmit_IT(&huart4, (uint8_t *)str, sizeof(str));
  //   // s         = HAL_UART_Transmit(&huart4, (uint8_t *)str, sizeof(str), 300);
  //   // __HAL_UART_FLUSH_DRREGISTER(&huart4);
  //   // __HAL_UART_CLEAR_OREFLAG(&huart4);
  //   // s = HAL_UART_Receive(&huart4, (uint8_t *)rx_buff, sizeof(rx_buff), 300);
  //   // log_info("UART4 data: " + std::string(rx_buff) + " status: " + std::to_string(s));
  //   // memset(rx_buff, 0, sizeof(rx_buff));
  //   // // HAL_UART_Transmit_IT(&huart4, (uint8_t *)str, sizeof(str));
  //   // // HAL_StatusTypeDef s = HAL_UART_Receive_IT(&huart4, (uint8_t *)rx_buff, sizeof(rx_buff));
  //   // // s                   = HAL_UART_Receive(&huart4, (uint8_t *)data, sizeof(data), 300);
  //   HAL_Delay(300);
  // }


  // log_debug("UART4 data: " + std::string(data));


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