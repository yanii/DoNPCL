add_executable(example_pcl_don example_pcl_don.cpp)
target_link_libraries(example_pcl_don don ${Boost_LIBRARIES} ${PCL_LIBRARIES})

if(CUDA_FOUND)
	add_executable(example_pcl_don_gpu example_pcl_don_gpu.cpp)
	target_link_libraries(example_pcl_don_gpu don ${Boost_LIBRARIES} ${PCL_LIBRARIES})
endif(CUDA_FOUND)
