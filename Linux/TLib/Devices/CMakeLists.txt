cmake_minimum_required(VERSION 3.25)

project(DevicesLib)

add_library(DevicesLib  ${CMAKE_CURRENT_LIST_DIR}/Source/UART.cpp
                        ${CMAKE_CURRENT_LIST_DIR}/Source/GPIO.cpp)
