#pragma once

#ifdef CONFIG_MYL_UTILS_LOG
#include "myl_utils/zephyr/log.h"
#else

// ---- Non-Zephyr fallback: printf-based logging stubs ----
#include <cstdio>

#ifndef LOG_ERR
#define LOG_ERR(...) do { fprintf(stderr, "[ERR] " __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#endif

#ifndef LOG_WRN
#define LOG_WRN(...) do { fprintf(stderr, "[WRN] " __VA_ARGS__); fprintf(stderr, "\n"); } while (0)
#endif

#ifndef LOG_INF
#define LOG_INF(...) do { printf("[INF] " __VA_ARGS__); printf("\n"); } while (0)
#endif

#ifndef LOG_DBG
#define LOG_DBG(...) do { printf("[DBG] " __VA_ARGS__); printf("\n"); } while (0)
#endif

#ifndef INITIALIZE_MYL_UTILS_LOG
#define INITIALIZE_MYL_UTILS_LOG()
#endif

#ifndef DECLARE_MYL_UTILS_LOG
#define DECLARE_MYL_UTILS_LOG()
#endif

#endif