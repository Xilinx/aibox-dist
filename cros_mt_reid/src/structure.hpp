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

#ifndef _FTD_STRUCTURE_HPP
#define _FTD_STRUCTURE_HPP

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vitis/ai/reid.hpp>
#include "hungarian.hpp"
#include "trajectory.hpp"
typedef pair<int, Mat> imagePair;
class paircomp {
 public:
  bool operator()(const imagePair& n1, const imagePair& n2) const {
    if (n1.first == n2.first) return n1.first > n2.first;
    return n1.first > n2.first;
  }
};

namespace vitis {
namespace ai {

double cosine_distance(Mat feat1, Mat feat2);

double get_euro_dis(Mat feat1, Mat feat2);

class FTD_Structure {
 public:
  FTD_Structure(const Mat& transform, int cam);
  ~FTD_Structure();
  void clear();

  std::vector<OutputCharact> Update(uint64_t frame_id,
                                    std::vector<InputCharact>& input_characts);
  std::vector<int> GetRemoveID();

  int max_age = 10000;
  int min_hits = 3;
  int frame_count = 0;
  std::vector<std::shared_ptr<FTD_Trajectory>> tracks;

 private:
  Mat transform_;
  int cam;
  float iou_threshold;
  float feat_distance_low;
  float feat_distance_high;
  float score_threshold;
  cv::Rect_<float> roi_range;

  void GetOut(std::vector<OutputCharact>& output_characts);
  std::vector<int> remove_id_this_frame;
};

}  // namespace ai
}  // namespace vitis
#endif
