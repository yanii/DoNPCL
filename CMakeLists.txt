cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(DoNPCL)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#Note: Eclipse automatically picks up include paths with this on!
SET(CMAKE_VERBOSE_MAKEFILE ON)

##### Subdirectories #####
add_subdirectory(cmake)
add_subdirectory(src)

##### Package Dependencies #####
include(UseDoxygen OPTIONAL)
