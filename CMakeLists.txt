# common/CMakeLists.txt

cmake_minimum_required(VERSION 3.20)

# ---------------------------------------------------------------------------
# Platform detection: Zephyr vs. standalone (STM32 HAL, bare-metal, etc.)
# ---------------------------------------------------------------------------
# When used as a Zephyr module the variable ZEPHYR_BASE (or the Zephyr
# package) is already available; otherwise we build as a plain CMake library.
# ---------------------------------------------------------------------------

if(DEFINED ZEPHYR_BASE OR DEFINED CONFIG_ZEPHYR_ENABLE_CMSIS_CORE_HEADERS)
  # ---- Zephyr build ----
  find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
  set(MYL_UTILS_PLATFORM "zephyr" CACHE INTERNAL "")
else()
  set(MYL_UTILS_PLATFORM "standalone" CACHE INTERNAL "")
endif()

project(myl_utils VERSION 1.0.0
        DESCRIPTION "myl_utils library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_sources(${PROJECT_NAME}
    PRIVATE
        src/dummy.cpp
)

if(MYL_UTILS_PLATFORM STREQUAL "zephyr")
  # Zephyr: logging.cpp uses Zephyr log macros, compiled into the app target
  target_sources(app
      PRIVATE
          src/logging.cpp
  )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC include)

# ---------------------------------------------------------------------------
# Standalone (non-Zephyr) defaults
# ---------------------------------------------------------------------------
if(MYL_UTILS_PLATFORM STREQUAL "standalone")
  target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
endif()
