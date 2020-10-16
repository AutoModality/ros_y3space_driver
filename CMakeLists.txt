cmake_minimum_required(VERSION 2.8.3)
project(y3space_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             roscpp
             rospy
             sensor_msgs
             serial
             std_msgs)

find_package(Boost REQUIRED COMPONENTS system)

###################
## catkin Config ##
###################

include_directories(include/y3space_driver)

catkin_package(CATKIN_DEPENDS geometry_msgs
                              roscpp
                              rospy
                              sensor_msgs
                              serial
                              std_msgs)

###########
## Build ##
###########

include_directories(
    INCLUDE_DIRS include
    ${BOOST_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_library(serial_interface
            src/y3space_driver/SerialInterface.cpp)

add_executable(y3space_driver
               src/y3space_driver/Y3SpaceDriver.cpp
               src/y3space_driver/main.cpp)

target_link_libraries(y3space_driver
                      serial_interface
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})


#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS y3space_driver serial_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

#Mark cpp header files for installation
install(DIRECTORY include/y3space_driver
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
