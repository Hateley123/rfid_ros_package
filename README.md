# Package Introduction
This package is designed to create a ros2 topic that can be used to pass information from an Impinj RFID Reader to the computer for later use. Since most of the script operates off of python to increase computation efficiency, this publication of a ros topic prevents the need of using intermediary files for data transfer.

# Dependencies
This system relies upon ROS2 Humble (https://docs.ros.org/en/humble/Installation.html), Ubuntu 22 and ros2_dotnet package (https://github.com/ros2-dotnet/ros2_dotnet/tree/main). So prior to installation of this package both of those will need to be installed. The instructions for each can be found on the websites previously linked.

# Installation Instructions
The following instructions are just for installing the package. You cannot build the package until you follow the modification instructions. The files in this package were uploaded in path locations that are specific to my system. So the paths will need to be modified. Make sure it is installed in the same workspace the ros2_dotnet package or the system will not work.
```
cd /path/to/workspace/src/
source /opt/ros/humble/setup.bash
git clone https://github.com/Hateley123/rfid_ros_package.git
```

# Modification Instructions
ensure to change the following code blocks in the cmakelist.txt

```
set(CS_SOURCES
  /path/to/workspace/src/rfid_ros_package/src/csharp/RFID_Tag_Publisher.cs
)

set(DLL_DIR /path/to/workspace/src/rfid_ros_package/libraries)

add_dotnet_executable(rfid_ros_node
  /path/to/workspace/src/rfid_ros_package/src/csharp/RFID_Tag_Publisher.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

add_library(${PROJECT_NAME}_native SHARED
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_action_client.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_client.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_clock.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_guard_condition.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_node.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_publisher.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_timer.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_qos_profile.c
  /path/to/workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet.c
)

```
Once these modifications have been made your system will be able to buil properly.

# Building Instructions

You cannot use a basic `colcon build` for this topic otherwise the ros node will not be created properly.

```
cd /path/to/workspace/
colcon build --symlink-install
```

# Launching Instructions

```
ros2 run rfid_ros_package rfid_ros_node
```

# Topic Layout 
C# has an issue importing custom made topics into the .cs file. Therefore a standard msg topic format had to be used in order to pass the information from the rfid reader to ros. In this system, the data that is being transferred is being used for Phase difference of arrival calculations and Angle of arrival calculations. Therefore, the phase values from the centeral antenna at two different frequencies are stored, and the phase values from each of the antennas themselves is stored. The topic is organized as follows `[frequency_1_phase,frequency_2_phase,antenna_1_phase,antenna_2_phase,antenna_3_phase]`
