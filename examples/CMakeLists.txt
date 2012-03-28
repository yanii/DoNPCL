add_executable(example_pcl_don example_pcl_don.cpp)
target_link_libraries(example_pcl_don don ${Boost_LIBRARIES} ${PCL_LIBRARIES})

if(CUDA_FOUND)
	add_executable(example_pcl_dongpu example_pcl_dongpu.cpp)
	target_link_libraries(example_pcl_dongpu don ${Boost_LIBRARIES} ${PCL_LIBRARIES})
endif(CUDA_FOUND)
