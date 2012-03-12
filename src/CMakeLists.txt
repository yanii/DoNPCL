ADD_LIBRARY(don don.cpp)
TARGET_LINK_LIBRARIES(don ${PCL_LIBRARIES})

add_executable(pcl_don pcl_don.cpp)
target_link_libraries(pcl_don don ${Boost_LIBRARIES} ${PCL_LIBRARIES})
