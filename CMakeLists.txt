cmake_minimum_required(VERSION 3.5)

if (NOT PROJECT_VERSION)
set(PROJECT_VER "1.0.0-devel")
else()
set(PROJECT_VER "${PROJECT_VERSION}")
endif()


file(REMOVE ${CMAKE_SOURCE_DIR}/sdkconfig)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
	set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfig.release.in")
else()
	set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfig.debug.in")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(fanboy)

if (NOT ${IDF_VER} MATCHES 4)
	message(FATAL_ERROR "IDF 4.x.x required")
endif()
