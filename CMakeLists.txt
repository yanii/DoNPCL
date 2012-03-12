cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(DoNPCL)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#Note: Eclipse automatically picks up include paths with this on!
SET(CMAKE_VERBOSE_MAKEFILE ON)

##### Package Dependencies #####
include(UseDoxygen OPTIONAL)

find_package(PCL 1.4 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

MESSAGE("PCL information:") 
MESSAGE("  PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}") 
MESSAGE("  PCL_LIBRARIES: ${PCL_LIBRARIES}") 
MESSAGE("  PCL_DEFINITIONS: ${PCL_DEFINITIONS}") 

set(BOOST_LIBS program_options)
FIND_PACKAGE(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 

INCLUDE_DIRECTORIES(include)

##### Subdirectories #####
add_subdirectory(cmake)
add_subdirectory(src)
