#include "main.hpp"
#include "main_prog.hpp"
#include "Timing.hpp"
#include "simple_task.hpp"
#include "fdcan.hpp"

namespace se = stmepic;


/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM7 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM6) {
    se::Ticker::get_instance().irq_update_ticker();
  }

  if(htim->Instance == TIM7) {
    HAL_IncTick();
  }
}


se::SimpleTask task_blink;


void task_blink_func(se::SimpleTask &task, void *pvParameters) {
  while(1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    vTaskDelay(1000);
  }
}


void main_prog() {
  // task_blink.task_init(task_blink_func,nullptr,100,nullptr);
  // task_blink.task_run();
  FDCAN_FilterTypeDef sFilterConfig;
  auto mayby_fdcan = se::FDCAN::Make(hfdcan1,sFilterConfig,nullptr,nullptr);
  STMEPIC_ASSING_OR_HRESET(fdcan,mayby_fdcan);
  fdcan->hardware_start();

  HAL_GPIO_TogglePin(USER_LED_2_GPIO_Port, USER_LED_2_Pin);

  while(true) {
    HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
    HAL_GPIO_TogglePin(USER_LED_2_GPIO_Port, USER_LED_2_Pin);
    HAL_Delay(100);
    se::CanDataFrame frame;
    frame.extended_id = false;
    frame.frame_id  = 0x123;
    frame.data[0] = 0x12;
    frame.remote_request = false;
    fdcan->write(frame);
  }


  // se::Ticker::get_instance().init(&htim6);

  // Your code here like your tasks, drivers, etc.
  // Do not start FreeRTOS kernel here since it will be start later in main.cpp
}