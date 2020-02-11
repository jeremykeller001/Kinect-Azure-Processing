cmake_minimum_required( VERSION 3.15 )
 
# Create Project
project( solution )
set( SOURCE
	src/IOUtils.cpp
	src/KinectAzureUtils.cpp
	src/main.cpp
	src/MatrixUtils.cpp
	src/PclUtils.cpp
	src/Ply.cpp)

set (HEADERS
	src/IOUtils.h
	src/KinectAzureUtils.h
	src/MatrixUtils.h
	src/PclUtils.h
	src/Ply.h)

add_executable( project ${SOURCE} ${HEADERS} )
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT project )
set_property (TARGET project
  PROPERTY
    # Enable C++17 standard compliance
    CXX_STANDARD 17
)
 
# Find Packages
find_package( PCL 1.9 REQUIRED )
 
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
  target_link_libraries( project ${PCL_LIBRARIES} )
endif()

set(KINECT_SDK_INCLUDE "C:/Program Files/Azure Kinect SDK v1.2.0/sdk/include/")
set(EIGEN_INCLUDE "F:/Software/eigen-3.3.7/")
include_directories( ${KINECT_SDK_INCLUDE} ${EIGEN_INCLUDE})

set(KINECT_SDK_LIB64 "C:/Program Files/Azure Kinect SDK v1.2.0/sdk/windows-desktop/amd64/release/lib")
target_link_libraries( project ${KINECT_SDK_LIB64}/k4a.lib ${KINECT_SDK_LIB64}/k4arecord.lib )
