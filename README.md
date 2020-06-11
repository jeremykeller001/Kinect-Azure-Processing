# Kinect-Azure-Processing

Kinect Azure Processing is a C++ library for processing depth data from the Microsoft Kinect. It can generate joint tracking output, combined points clouds, and meshing output. 

## Installation
This code was developed and tested using Visual Studio 2019 on Windows 10
### System Requirements:
Nvidia CUDA Toolkit 10.2 (with compatible Nvidia GPU)

Point Cloud Library 1.10.1

Azure Kinect Sensor SDK 1.3.0 (or higher)

Azure Kinect Body Tracking SDK 1.0.1 (or higher)

CMake 3.15 (or higher)

### Setup:
Ensure .dll's for all libraries are added to the PATH environmental variable.

Add references to the include and library files for the Kinect Libraries in the CMakeLists.txt file (they are currently set to the default install location).


## Usage
Run "KinectAzureProcessor.exe -h" for up to date usage guide.

A calibration file is required for joint tracking and multi camera setups. See /examples for a sample file with instructions on how to configure one.


## License
[MIT](https://choosealicense.com/licenses/mit/)
