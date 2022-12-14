/*
 * Copyright 2019 Xilinx Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _FTD_FILTER_LINEAR_HPP_
#define _FTD_FILTER_LINEAR_HPP_
/// #define _FTD_DEBUG_
#include <array>
#include <opencv2/core.hpp>
#include <vector>

namespace vitis {
namespace ai {

class FTD_Filter_Linear {
 public:
  FTD_Filter_Linear(){};
  ~FTD_Filter_Linear(){};
  void Init(const cv::Rect_<float> &bbox);
  void UpdateDetect(const cv::Rect_<float> &bbox);
  void UpdateReidTracker(const cv::Rect_<float> &bbox);
  void UpdateFilter();
  cv::Rect_<float> GetPre();
  cv::Rect_<float> GetPost();

 private:
  void LeastSquare(std::vector<std::array<double, 2>> &coord,
                   std::array<double, 8> &para, double x, int region);
  void ClearSquare(std::vector<std::array<double, 2>> &coord,
                   std::array<double, 8> &para, double step);
  void LeastMean(std::vector<std::array<double, 2>> &coord,
                 std::array<double, 4> &para, double x, int region);
  void ClearMean(std::vector<std::array<double, 2>> &coord,
                 std::array<double, 4> &para, double step);
  std::vector<std::array<double, 2>> coordx;
  std::vector<std::array<double, 2>> coordy;
  std::vector<std::array<double, 2>> coords;
  std::vector<std::array<double, 2>> coordr;
  std::array<double, 8> parax;
  std::array<double, 8> paray;
  std::array<double, 8> paras;
  std::array<double, 4> parar;
  float frame_id;
  float frame_start;
  float frame_max;
  std::array<int, 4> allregion;
};

}  // namespace ai
}  // namespace vitis
#endif
