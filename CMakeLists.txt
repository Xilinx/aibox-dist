#
# Copyright 2019 Xilinx Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.5)

project(aibox-dist)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -fpermissive -rdynamic -Wl,--no-undefined" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-error=cpp" ) # disable the warning message from gstreamer rtsp libs
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++17")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0")
SET(CMAKE_C_FLAGS_DEBUG "-ggdb3 -O0")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(GStreamer REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS opencv_core opencv_video opencv_videoio opencv_imgproc opencv_imgcodecs opencv_highgui)
#find_package(cppzmq REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}) 
SET(INSTALL_PATH "opt/xilinx")
#SET(CMAKE_INSTALL_PREFIX ${INSTALL_PATH})


add_subdirectory(cros_mt_reid/)

add_library(aibox_dist_reid SHARED src/aibox_dist_reid.cpp src/common.hpp)
target_include_directories(aibox_dist_reid PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_include_directories(aibox_dist_reid PRIVATE reidtracker/include/)
set_target_properties(aibox_dist_reid PROPERTIES INSTALL_RPATH "/opt/xilinx/lib")
target_link_libraries(aibox_dist_reid
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 gstrtsp-1.0 gstrtspserver-1.0
  glib-2.0 gobject-2.0 ${OpenCV_LIBS} jansson vitis_ai_library-reid gstvvasinfermeta-1.0 glog pthread
  vitis_ai_library-classification vitis_ai_library-dpu_task)
install(TARGETS aibox_dist_reid DESTINATION ${INSTALL_PATH}/lib)
add_dependencies(aibox_dist_reid cros_mt_reid)

add_library(aibox_dist_crop SHARED src/aibox_dist_crop.cpp)
target_include_directories(aibox_dist_crop PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(aibox_dist_crop
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 gstrtsp-1.0 gstrtspserver-1.0
  glib-2.0 gobject-2.0 ${OpenCV_LIBS} jansson vvasutil gstvvasinfermeta-1.0 glog)
install(TARGETS aibox_dist_crop DESTINATION ${INSTALL_PATH}/lib)

add_library(aibox_dist_drawreid SHARED src/aibox_dist_drawreid.cpp)
target_include_directories(aibox_dist_drawreid PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(aibox_dist_drawreid
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 gstrtsp-1.0 gstrtspserver-1.0
  glib-2.0 gobject-2.0 ${OpenCV_LIBS}  jansson vvasutil gstvvasinfermeta-1.0)
install(TARGETS aibox_dist_drawreid DESTINATION ${INSTALL_PATH}/lib)

add_library(aibox_dist_pedpp SHARED src/aibox_dist_pedpp.c)
target_include_directories(aibox_dist_pedpp PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(aibox_dist_pedpp
  jansson vvasutil gstvvasinfermeta-1.0)
install(TARGETS aibox_dist_pedpp DESTINATION ${INSTALL_PATH}/lib)

add_library(gstaibox-dist-attach SHARED src/gstaibox_dist_attach.cpp src/dataSaveLoad.cpp)
target_include_directories(gstaibox-dist-attach PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(gstaibox-dist-attach
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 
  jansson vvasutil gstvvasinfermeta-1.0 ${OpenCV_LIBS} boost_serialization
  zmq)
install(TARGETS gstaibox-dist-attach DESTINATION ${INSTALL_PATH}/lib)

add_library(gstaibox-dist-parse SHARED src/gstaibox_dist_parse.cpp src/dataSaveLoad.cpp)
target_include_directories(gstaibox-dist-parse PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(gstaibox-dist-parse
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 
  jansson vvasutil gstvvasinfermeta-1.0 ${OpenCV_LIBS} boost_serialization
  zmq)
install(TARGETS gstaibox-dist-parse DESTINATION ${INSTALL_PATH}/lib)

add_library(gstaibox-dist-sum SHARED src/gstaibox_dist_sum.cpp src/cross_track.cpp )
target_include_directories(gstaibox-dist-sum PRIVATE ${GSTREAMER_INCLUDE_DIRS} cros_mt_reid/src/)
target_link_libraries(gstaibox-dist-sum
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 
  jansson vvasutil gstvvasinfermeta-1.0 ${OpenCV_LIBS} cros_mt_reid)
install(TARGETS gstaibox-dist-sum DESTINATION ${INSTALL_PATH}/lib)


add_executable(aibox-dist-cam src/cam.cpp)
target_include_directories(aibox-dist-cam PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(aibox-dist-cam
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 gstrtsp-1.0 gstrtspserver-1.0
  glib-2.0 gobject-2.0 )
install(TARGETS aibox-dist-cam DESTINATION ${INSTALL_PATH}/bin/aibox-dist)

add_executable(aibox-dist-monitor src/monitor.cpp)
target_include_directories(aibox-dist-monitor PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(aibox-dist-monitor
  gstapp-1.0 gstreamer-1.0 gstbase-1.0 gobject-2.0 glib-2.0 gstvideo-1.0 gstallocators-1.0 gstrtsp-1.0 gstrtspserver-1.0
  glib-2.0 gobject-2.0 ${OpenCV_LIBS})
install(TARGETS aibox-dist-monitor DESTINATION ${INSTALL_PATH}/bin/aibox-dist)

install(PROGRAMS
    script/aibox-dist-install.py
    script/aibox-dist-monitor
    script/aibox-dist-cam
    DESTINATION ${INSTALL_PATH}/bin)



install(FILES
    README
    DESTINATION ${INSTALL_PATH}/
    RENAME      README_AIBOX-DIST
    )

install(FILES
    config/ped_pp.json
    config/refinedet.json
    config/crop.json
    config/reid.json
    config/draw_reid.json
    config/cam_setup.json
    DESTINATION ${INSTALL_PATH}/share/vvas/${CMAKE_PROJECT_NAME})

install(DIRECTORY
    notebook/
    DESTINATION ${INSTALL_PATH}/share/notebooks/${CMAKE_PROJECT_NAME}/)

set(VERSION "1.0.1")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_GENERATOR "RPM")
set(CPACK_PACKAGE_NAME "${CMAKE_PROJECT_NAME}")
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_RPM_PACKAGE_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
set(CPACK_PACKAGING_INSTALL_PREFIX "/")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_RPM_SPEC_MORE_DEFINE "%define _build_id_links none")
include(CPack)
