# common/CMakeLists.txt

cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(myl_utils VERSION 1.0.0
        DESCRIPTION "myl_utils library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_sources(${PROJECT_NAME}
    PRIVATE
        src/dummy.cpp
)
target_sources(app
    PRIVATE
        src/logging.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC include)
