ADD_LIBRARY(don SHARED don.cpp ${CMAKE_SOURCE_DIR}/include/pcl/features/impl/don.hpp ${CMAKE_SOURCE_DIR}/include/pcl/features/don.h)
TARGET_LINK_LIBRARIES(don ${PCL_LIBRARIES})

INSTALL(TARGETS don
  RUNTIME DESTINATION bin COMPONENT app
  LIBRARY DESTINATION lib COMPONENT runtime
  ARCHIVE DESTINATION lib COMPONENT runtime
  PUBLIC_HEADER DESTINATION include COMPONENT dev
  FRAMEWORK DESTINATION "/Library/Frameworks"
)