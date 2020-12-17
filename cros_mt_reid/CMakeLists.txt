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
project(cros_mt_reid VERSION 1.3.0 LANGUAGES C CXX)

include(${PROJECT_SOURCE_DIR}/cmake/XilinxCommon.cmake)

option(BUILD_TEST "build test" ON)

include_directories(${CMAKE_SOURCE_DIR}/include)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_LIST_DIR}/cmake")
set(INSTALL_LIB_DIR ${INSTALL_PATH}/lib)
set(INSTALL_BIN_DIR ${INSTALL_PATH}/bin)
set(INSTALL_INCLUDE_DIR ${INSTALL_PATH}/include)
set(INSTALL_CMAKE_DIR ${INSTALL_PATH}/share/cmake/${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-error=cpp" )

include_directories("${INSTALL_PATH}/include")
link_directories("${INSTALL_PATH}/lib")
find_package(Protobuf 3.4.0 REQUIRED)
find_package(OpenCV COMPONENTS opencv_core opencv_video opencv_videoio opencv_imgproc opencv_imgcodecs opencv_highgui)
if(NOT OpenCV_FOUND)
  find_package(OpenCV_LIBS opencv_core opencv_video opencv_videoio opencv_imgproc opencv_imgcodecs opencv_highgui)
endif(NOT OpenCV_FOUND)


find_package(vart COMPONENTS REQUIRED)

add_subdirectory(src)

#if(BUILD_TEST)
#  add_subdirectory(test)
#endif()

set(CPACK_PACKAGE_NAME "libreidtracker")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "reid tracker")
set(CPACK_PACKAGE_CONTACT "Yongsheng Xu<xysheng@xilinx.com>")
set(CPACK_PACKAGING_INSTALL_PREFIX "/")

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_RPM_PACKAGE_ARCHITECTURE "aarch64")
set(CPACK_DEB_COMPONENT_INSTALL False)
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_ARCHIVE_TYPE "gnutar")
set(CPACK_DEBIAN_COMPRESSION_TYPE "gzip")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "vitis_ai_library (>=1.1.0)")

set(CPACK_RPM_COMPONENT_INSTALL False)
set(CPACK_RPM_PACKAGE_LICENSE "Apache")
set(CPACK_RPM_PACKAGE_GROUP "Development/Tools")
set(CPACK_RPM_COMPRESSION_TYPE "xz")
set(CPACK_RPM_PACKAGE_REQUIRES "vitis_ai_library >= 1.1.0")

include(CPack)
