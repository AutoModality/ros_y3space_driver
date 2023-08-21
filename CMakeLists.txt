cmake_minimum_required(VERSION 3.5)
project(y3space_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
        rclcpp
        std_msgs
        std_srvs
        sensor_msgs
        geometry_msgs
        rosbag2_storage
        rosbag2_cpp
        am_utils
        serial
      )

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()



###########
## Build ##
###########

include_directories(
      include
    )


add_executable(y3space_driver
                src/y3space_driver/SerialInterface.cpp
                src/y3space_driver/Y3SpaceDriver.cpp
                src/y3space_driver/driver_node.cpp)
ament_target_dependencies(y3space_driver ${dependencies})



#############
## Install ##
#############
install(DIRECTORY include/
        DESTINATION include/
    )
    
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}
    )
install(TARGETS
  y3space_driver
  DESTINATION lib/${PROJECT_NAME})

#############
##  Test   ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${dependencies}
  )
  #target_link_libraries(${PROJECT_NAME}test name_of_local_library)
endif()


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()