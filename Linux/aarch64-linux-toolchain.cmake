set(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 3.12)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

#----------------------------------------------------------------------------------------------
set(TOOLCHAINS $ENV{HOME}/toolchains)

set(TOOLCHAIN_NAME linaro-toolchain-latest-latest-7)

if (EXISTS ${TOOLCHAINS})
	if (EXISTS ${TOOLCHAINS}/${TOOLCHAIN_NAME})
		set(TOOLCHAIN_ROOT ${TOOLCHAINS}/linaro-toolchain-latest-latest-7)
	else()
		message(FATAL_ERROR "No toolchains found! Check if : ${TOOLCHAIN_NAME} is exist in ${TOOLCHAINS}")
	endif()
else()
	message(FATAL_ERROR "No toolchains dir found! Check if ${TOOLCHAINS} is exist")
		
endif()

message(STATUS "Toolchain ${TOOLCHAINS}/${TOOLCHAIN_NAME} is using")

#----------------------------------------------------------------------------------------------

set(COMPILER_PREFIX ${TOOLCHAINS}/${TOOLCHAIN_NAME}/bin/aarch64-linux-gnu-)


set(CMAKE_C_COMPILER   ${COMPILER_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${COMPILER_PREFIX}g++)
set(CMAKE_GNU_STRIPPER ${COMPILER_PREFIX}strip)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")

set(CMAKE_LIBRARY_ARCHITECTURE "aarch64-linux-gnu")

set(CMAKE_CXX_COMPILER_ID GNU)

#----------------------------------------------------------------------------------------------

set(CMAKE_FIND_ROOT_PATH ${TOOLCHAINS}/${TOOLCHAIN_NAME}/aarch64-linux-gnu/)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
