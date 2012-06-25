add_executable(example_pcl_don example_pcl_don.cpp)
target_link_libraries(example_pcl_don don ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(example_pcl_don_kinect example_pcl_don_kinect.cpp)
target_link_libraries(example_pcl_don_kinect don ${Boost_LIBRARIES} ${PCL_LIBRARIES})

INSTALL(TARGETS example_pcl_don example_pcl_don_kinect
  RUNTIME DESTINATION bin COMPONENT app
  LIBRARY DESTINATION lib COMPONENT runtime
  ARCHIVE DESTINATION lib COMPONENT runtime
  PUBLIC_HEADER DESTINATION include COMPONENT dev
  FRAMEWORK DESTINATION "/Library/Frameworks"
)

if(CUDA_FOUND)
	add_executable(example_pcl_don_gpu example_pcl_don_gpu.cpp)
	target_link_libraries(example_pcl_don_gpu don ${Boost_LIBRARIES} ${PCL_LIBRARIES})
	
	INSTALL(TARGETS example_pcl_don_gpu
	  RUNTIME DESTINATION bin COMPONENT app
	  LIBRARY DESTINATION lib COMPONENT runtime
	  ARCHIVE DESTINATION lib COMPONENT runtime
	  PUBLIC_HEADER DESTINATION include COMPONENT dev
	  FRAMEWORK DESTINATION "/Library/Frameworks"
	)
endif(CUDA_FOUND)
