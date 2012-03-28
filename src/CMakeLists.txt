ADD_LIBRARY(don don.cpp ${CMAKE_SOURCE_DIR}/include/pcl/features/impl/don.hpp ${CMAKE_SOURCE_DIR}/include/pcl/features/don.h)
TARGET_LINK_LIBRARIES(don ${PCL_LIBRARIES})
