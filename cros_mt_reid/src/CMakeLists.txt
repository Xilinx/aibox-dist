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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SYSROOT}/usr/share/cmake/")
include(XilinxCommon)
include(XilinxVersion)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
set(OpenCV_LIBS opencv_core opencv_video opencv_videoio opencv_imgproc opencv_imgcodecs opencv_highgui)
add_library(${PROJECT_NAME} SHARED
  filter_linear.cpp  structure.cpp  trajectory.cpp
  filter_linear.hpp  structure.hpp  trajectory.hpp
  hungarian.cpp
  hungarian.hpp
  mtmc_reid.hpp
  mtmc_reid.cpp
  calibration.cpp
  calibration.hpp
  parse_json_config.cpp
  parse_json_config.hpp
  ${CMAKE_CURRENT_BINARY_DIR}/version.c
  )

target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS} vitis_ai_library-reid pthread glog jansson)
set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_NAME  ${PROJECT_NAME}
  )
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR})

#install(
#  EXPORT ${PROJECT_NAME}-targets
#  NAMESPACE ${PROJECT_NAME}::
#  DESTINATION ${INSTALL_CMAKE_DIR})

#install(FILES ../include/vitis/ai/reidtracker.hpp
#  DESTINATION include/vitis/ai/)
