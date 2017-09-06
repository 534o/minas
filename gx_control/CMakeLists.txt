cmake_minimum_required(VERSION 2.8.3)
project(gx_control)

find_package(catkin REQUIRED COMPONENTS ethercat_manager soem message_generation)

########################
## message generation ##
########################

add_message_files(
  DIRECTORY msg
  FILES DigitalIO.msg)

generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gx_client
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(gx_client src/gx_client.cpp)

add_library(gx_driver_controller src/gx_driver_controller.cpp)
add_dependencies(gx_driver_controller ${PROJECT_NAME}_generate_messages_cpp)

## Declare a C++ executable
add_executable(slaveinfo src/slaveinfo.cpp)
add_executable(simple_test src/simple_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gx_driver_controller ${catkin_LIBRARIES} gx_client)
target_link_libraries(slaveinfo ${catkin_LIBRARIES} gx_client)
target_link_libraries(simple_test ${catkin_LIBRARIES} gx_client)

function(set_sudo_binary tgt)
  get_target_property(loc ${tgt} "LOCATION")
  string(ASCII 27 ESCAPE)
  add_custom_target(sudo_${tgt}
    COMMAND sudo -n sh -c 'setcap cap_net_raw,cap_ipc_lock=+ep ${loc}' || (>&2 echo "${ESCAPE}[31m#\ type\ following\ command\ before\ execute\ reset...\ sudo\ setcap cap_net_raw,cap_ipc_lock=+ep\ ${loc}${ESCAPE}[0m")
    DEPENDS bin/${tgt})
  add_custom_target(sudo_${tgt}_all ALL DEPENDS sudo_${tgt})
endfunction()
set_sudo_binary(slaveinfo)
set_sudo_binary(simple_test)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS slaveinfo simple_test gx_client gx_driver_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# https://github.com/tork-a/gx/issues/20 / http://stackoverflow.com/questions/3352041/creating-binary-with-cmake-removes-runtime-path
set_property(TARGET slaveinfo PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_property(TARGET simple_test PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)
set_property(TARGET gx_client PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES gx_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(status "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()

#############
## Release ##
#############

# debbuild
# catkin build gx_control --make-args debbuild
# ROS_PARALLEL_JOBS='' catkin_make --pkg gx_control -- debbuild
add_custom_target(debbuild_${PROJECT_NAME}
  COMMAND sudo -n true || (>&2 echo "debbuild need sudo access to make debs" && exit 1)
  COMMAND bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
  COMMAND MAKEFLAGS= fakeroot debian/rules binary
  COMMAND find debian obj-x86_64-linux-gnu -type f -not -name postinst -print0 | xargs -0 rm --
  COMMAND find debian obj-x86_64-linux-gnu -type d -empty -delete
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
