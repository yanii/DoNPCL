ADD_LIBRARY(don don.cpp ${CMAKE_SOURCE_DIR}/include/pcl/features/impl/don.hpp ${CMAKE_SOURCE_DIR}/include/pcl/features/don.h)
TARGET_LINK_LIBRARIES(don ${PCL_LIBRARIES})

add_executable(pcl_don pcl_don.cpp pcl_don.h)
target_link_libraries(pcl_don don ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_dongpu pcl_dongpu.cpp pcl_don.h)
target_link_libraries(pcl_dongpu don ${Boost_LIBRARIES} ${PCL_LIBRARIES})