cmake_minimum_required(VERSION 3.25)
project(Transmitter)

set(CMAKE_TOOLCHAIN_FILE aarch64-linux-toolchain.cmake)
set(CMAKE_CXX_STANDARD 17)

add_executable(Transmitter main.cpp
                            Source/Transmitter.cpp
)
target_include_directories(Transmitter PUBLIC TLib/Modulator/Include TLib/Devices/Include)

add_subdirectory(TLib)

target_link_libraries(Transmitter Modulator DevicesLib)