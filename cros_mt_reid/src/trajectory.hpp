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

#ifndef _FTD_TRAJECTORY_HPP_
#define _FTD_TRAJECTORY_HPP_

#include <set>
#include <tuple>
#include <vitis/ai/env_config.hpp>
#include <iostream>
#include "filter_linear.hpp"
DEF_ENV_PARAM(DEBUG_REID_TRACKER, "0")

using namespace std;
using namespace cv;

namespace vitis {
namespace ai {

typedef std::tuple<cv::Mat, int, cv::Rect_<float>, float, int, int>
    InputCharact;

typedef std::tuple<uint64_t, cv::Rect_<float>, float, int, int> OutputCharact;
using frame_rect = vector<pair<uint64_t, Rect_<float>>>;

class FTD_Trajectory : public std::enable_shared_from_this<FTD_Trajectory> {
 public:
  FTD_Trajectory();
  ~FTD_Trajectory();
  void Predict();
  int GetId();
  void SetId(uint64_t& update_id);
  InputCharact& GetCharact();
  void Init(Mat transImg, const InputCharact& input_charact,
             uint64_t frame, int cam);
  void UpdateTrack();
  void UpdateDetect(const InputCharact& input_charact, uint64_t frame);
  void UpdateWithoutDetect();
  void UpdateFeature(const cv::Mat& feat, int oridx);
  int GetStatus();
  bool GetShown();
  int getAge();
  int getHits();
  void DetectsAdd(uint64_t frame, Rect_<float>& val);
  Point2f to_world(Point2f point);
  frame_rect getDets();
  Point2f get_last_det_world();
  Point2f get_first_det_world();
  Point2f get_current_feet_worldpos();
  Rect_<float> get_state();
  OutputCharact GetOut();
  std::vector<cv::Mat> GetFeatures();
  std::vector<cv::Mat> GetFeatures(int oridx);
  int hits = 1;
  int hit_streak = 1;
  int age = 1;
  int time_since_update = 0;
  int id;
  int cam;
  InputCharact charact;
  frame_rect dets;
  bool SameAs(FTD_Trajectory& other);
  void BookSameTracker(FTD_Trajectory* ptr);
  cv::Mat feature;
  std::vector<cv::Mat> ori_feature;

 private:
  FTD_Filter_Linear filter;
  std::vector<cv::Mat> features;
  cv::Mat transformMat;
  int status;
  bool have_been_shown;
  std::set<void*> sameTrackers;
};

}  // namespace ai
}  // namespace vitis
#endif
