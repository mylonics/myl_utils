#pragma once

#ifdef CONFIG_MYL_UTILS_LOG
#include "myl_utils/zephyr/log.h"
#else

#error "MYL UTILS Logging module is only zephyr compatible.  Ensure CONFIG_MYL_UTILS_LOG is set in prj.conf"
// #ifndef LOG_ERR
//// #define LOG_ERR(...) Z_LOG(LOG_LEVEL_ERR, __VA_ARGS__)
// #error "LOG ERR Not implemented"
// #endif
//
// #ifndef LOG_WRN
// #define LOG_WRN(...) Z_LOG(LOG_LEVEL_ERR, __VA_ARGS__)
// #error "LOG WRN Not implemented"
// #endif
//
// #ifndef LOG_INF
// #define LOG_INF(...) Z_LOG(LOG_LEVEL_ERR, __VA_ARGS__)
// #error "LOG INF Not implemented"
// #endif
//
// #ifndef LOG_DBG
// #define LOG_DBG(...) Z_LOG(LOG_LEVEL_ERR, __VA_ARGS__)
// #error "LOG DBG Not implemented"
// #endif
//
// #ifndef INITIALIZE_MYL_UTILS_LOG
// #error "LOG Register Not implemented"
// #endif
//
// #ifndef DECLARE_MYL_UTILS_LOG
// #error "LOG Declare Not implemented"
// #endif

#endif