#pragma once
#include "main.hpp"
#include "stmepic.hpp"
#include "fdcan.hpp"

namespace se = stmepic;

void main_prog();

void can_callback_bmp280_get_status(se::CanBase &can, se::CanDataFrame &msg, void *args);
void can_callback_bmp280_get_data(se::CanBase &can, se::CanDataFrame &msg, void *args);