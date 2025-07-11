#pragma once

#include <zephyr/logging/log.h>

#define INITIALIZE_MYL_UTILS_LOG() LOG_MODULE_REGISTER(myl_utils, CONFIG_MYL_UTILS_LOG_LEVEL);
#define DECLARE_MYL_UTILS_LOG() LOG_MODULE_DECLARE(myl_utils, CONFIG_MYL_UTILS_LOG_LEVEL);
