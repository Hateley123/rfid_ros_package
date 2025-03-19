cmake_minimum_required(VERSION 3.8)
project(rfid_ros_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(dotnet_cmake_module REQUIRED)

set(CSHARP_TARGET_FRAMEWORK "net8.0")
find_package(DotNETExtra REQUIRED)
find_package(rcldotnet REQUIRED)
find_package(rcldotnet_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Enable C# message generation
set(ROSIDL_GENERATE_CSHARP ON CACHE BOOL "Generate C# messages")

# Define message files
set(MSG_FILES
  "msg/RFIDTagMsgs.msg"
)

# Generate message interfaces, including C#
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

# Set up C# source files
set(CS_SOURCES
  /home/rfid_tag_workspace/src/rfid_ros_package/src/csharp/RFID_Tag_Publisher.cs
)

# Define DLL directory for .NET dependencies
set(DLL_DIR /home/rfid_tag_workspace/src/rfid_ros_package/libraries)

# .NET dependencies (assemblies)
set(_assemblies_dep_dlls
  ${action_msgs_ASSEMBLIES_DLL}
  ${rcldotnet_ASSEMBLIES_DLL}
  ${builtin_interfaces_ASSEMBLIES_DLL}
  ${rcldotnet_common_ASSEMBLIES_DLL}
  ${std_msgs_ASSEMBLIES_DLL}
  ${std_srvs_ASSEMBLIES_DLL}
  ${unique_identifier_msgs_ASSEMBLIES_DLL}
  ${DLL_DIR}/Impinj.OctaneSdk.dll
  ${DLL_DIR}/Impinj.RShell.dll
  ${DLL_DIR}/LLRP.dll
  ${DLL_DIR}/LLRP.Impinj.dll
  ${DLL_DIR}/MathNet.Numerics.dll
  ${DLL_DIR}/Renci.SshNet.dll
)

# Add .NET executable
add_dotnet_executable(rfid_ros_node
  /home/rfid_tag_workspace/src/rfid_ros_package/src/csharp/RFID_Tag_Publisher.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

# Export the .NET assembly
ament_export_assemblies_dll("lib/${PROJECT_NAME}/dotnet/${PROJECT_NAME}_assemblies.dll")

# Install other files
install(
  DIRECTORY include/
  DESTINATION include
)

if(NOT WIN32)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
  endif()
endif()

# Handle native C++ library if needed
add_library(${PROJECT_NAME}_native SHARED
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_action_client.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_client.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_clock.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_guard_condition.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_node.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_publisher.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_timer.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet_qos_profile.c
  /home/rfid_tag_workspace/src/ros2_dotnet/ros2_dotnet/rcldotnet/rcldotnet.c
)

ament_target_dependencies(${PROJECT_NAME}_native
  "action_msgs"
  "builtin_interfaces"
  "unique_identifier_msgs"
  "rcl"
  "rcl_action"
  "rosidl_generator_c"
  "rosidl_typesupport_c"
)

ament_export_libraries(${PROJECT_NAME}_native)

install(TARGETS ${PROJECT_NAME}_native
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rcl)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_typesupport_c)

ament_package()
