#include "main.hpp"
#include "main_prog.hpp"
#include "Timing.hpp"
#include "encoder_magnetic.hpp"
#include "BNO055.hpp"
namespace se = stmepic;

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {
  if (htim->Instance == TIM6) {
    se::Ticker::get_instance().irq_update_ticker();
  } 

  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
 }



void main_prog(){
  se::sensors::imu::BNO055 imu(nullptr, nullptr, nullptr);
  // se::Ticker::get_instance().init(&htim6);

  // Your code here like your tasks, drivers, etc.
  // Do not start FreeRTOS kernel here since it will be start later in main.cpp
}