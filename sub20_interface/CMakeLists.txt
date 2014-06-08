cmake_minimum_required(VERSION 2.8.3)
project(sub20_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bosch_drivers_common
)


catkin_destinations()

# Download and build Sub20 library from Xdimax source
set(TARBALL_VERSION 110319)
set(TARBALL_MD5 b7612cbf99deb6b496e764366715579f)


find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED libusb-1.0)

set(libsub20_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/libsub20)
include(ExternalProject)
externalproject_add(libsub20
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
  URL http://www.xdimax.net/download/SUB-20-snap-${TARBALL_VERSION}.tgz
  URL_MD5 ${TARBALL_MD5}
  PATCH_COMMAND patch -p0 -t -N < ${CMAKE_CURRENT_SOURCE_DIR}/src/libsub20/libsub20.patch
  CONFIGURE_COMMAND ""
  BUILD_COMMAND $(MAKE) -C ${CMAKE_CURRENT_BINARY_DIR}/src/libsub20/lib libsub.so
  INSTALL_COMMAND mkdir -p ${libsub20_INCLUDE_DIRS} COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/src/libsub20/lib/libsub.h ${libsub20_INCLUDE_DIRS}/
)
set(libsub20_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/src/libsub20/lib/libsub.so)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${libsub20_INCLUDE_DIRS}
  LIBRARIES sub20_interface 
  CATKIN_DEPENDS bosch_drivers_common roscpp
  DEPENDS libusb
  EXPORTED_TARGETS libsub20
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${libsub20_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(sub20_interface
  src/sub20_interface.cpp
)
add_dependencies(sub20_interface
  libsub20
)
target_link_libraries(sub20_interface
  ${catkin_LIBRARIES}
  ${libsub20_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS sub20_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(FILES ${libsub20_LIBRARIES}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sub20_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
