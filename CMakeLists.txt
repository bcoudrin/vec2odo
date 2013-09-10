# Copyright (C) 2011 CNRS
#
# Author: Benjamin Coudrin
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#project(vec2odo)

SET(PROJECT_NAME vec2odo)
SET(PROJECT_DESCRIPTION "Test package for Odometry export in SoT")
SET(PROJECT_URL "")

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp rospy std_msgs tf message_generation)

#INCLUDE(cmake/base.cmake)
#INCLUDE(cmake/cpack.cmake)

# Where to compile shared objects
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_message_files(FILES vector6.msg)
add_service_files(FILES xyzrpy2odo.srv)
generate_messages(DEPENDENCIES nav_msgs std_msgs)

#SETUP_PROJECT()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vec2odo
  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf message_generation message_runtime
#  DEPENDS system_lib
)
include_directories(${catkin_INCLUDE_DIRS})

#ADD_REQUIRED_DEPENDENCY(urdf_parser_py >= 0.3)

ADD_EXECUTABLE(vec2odo_srv src/xyzrpy2odo_srv.cpp)
target_link_libraries(vec2odo_srv ${catkin_LIBRARIES})
add_dependencies(vec2odo_srv vec2odo_gencpp)

ADD_EXECUTABLE(vec2odo_topic src/odometry_translate_topic.cpp)
target_link_libraries(vec2odo_topic ${catkin_LIBRARIES})
add_dependencies(vec2odo_topic vec2odo_gencpp)

IF(NOT DEFINED CMAKE_INSTALL_LIBDIR)
    SET(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_PREFIX}/share)
ENDIF()
INCLUDE(cmake/GNUInstallDirs.cmake)
SET(CMAKE_INSTALL_FULL_PKGLIBDIR ${CMAKE_INSTALL_FULL_LIBDIR}/${PROJECT_NAME})
SET(CMAKE_INSTALL_PKGLIBDIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

message("Installation dir : " ${CMAKE_INSTALL_PKGLIBDIR})

INSTALL(TARGETS vec2odo_srv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CMAKE_INSTALL_PKGLIBDIR}
)

#SETUP_PROJECT_FINALIZE()
