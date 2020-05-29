cmake_minimum_required( VERSION 3.15 )
 
# Create Project
project( solution )
set( SOURCE
	src/BodyTrackingUtils.cpp
	src/IOUtils.cpp
	src/KinectAzureUtils.cpp
	src/main.cpp
	src/MatrixUtils.cpp
	src/PclUtils.cpp
	src/Ply.cpp)

set (HEADERS
	src/BodyTrackingUtils.h
	src/IOUtils.h
	src/KinectAzureUtils.h
	src/MatrixUtils.h
	src/PclUtils.h
	src/Ply.h)

add_executable( KinectAzureProcessor ${SOURCE} ${HEADERS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT KinectAzureProcessor )
set_property (TARGET KinectAzureProcessor
  PROPERTY
    # Enable C++14 standard compliance
    CXX_STANDARD 14
)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
 
# Find Packages
find_package( PCL 1.10.0 REQUIRED )
 
if( PCL_FOUND )
  # Additional Include Directories
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
 
  # Preprocessor Definitions
  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  #add_definitions( -DPCL_NO_PRECOMPILE )
 
  # Additional Library Directories
  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
 
  # Additional Dependencies
  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( KinectAzureProcessor ${PCL_LIBRARIES} )
endif()

set(KINECT_SDK_INCLUDE "C:/Program Files/Azure Kinect SDK v1.3.0/sdk/include/")
set(KINECT_BT_INCLUDE "C:/Program Files/Azure Kinect Body Tracking SDK/sdk/include")
include_directories( ${KINECT_SDK_INCLUDE} ${KINECT_BT_INCLUDE})

set(KINECT_SDK_LIB64 "C:/Program Files/Azure Kinect SDK v1.3.0/sdk/windows-desktop/amd64/release/lib")
set(KINECT_BT_LIB64 "C:/Program Files/Azure Kinect Body Tracking SDK/sdk/windows-desktop/amd64/release/lib")
target_link_libraries( KinectAzureProcessor ${KINECT_SDK_LIB64}/k4a.lib ${KINECT_SDK_LIB64}/k4arecord.lib ${KINECT_BT_LIB64}/k4abt.lib)
