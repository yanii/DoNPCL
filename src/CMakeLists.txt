ADD_LIBRARY(donfilter donfilter.cpp)
TARGET_LINK_LIBRARIES(donfilter ${PCL_LIBRARIES})

add_executable(pcl_don pcl_don.cpp)
target_link_libraries(pcl_don donfilter ${Boost_LIBRARIES} ${PCL_LIBRARIES})
