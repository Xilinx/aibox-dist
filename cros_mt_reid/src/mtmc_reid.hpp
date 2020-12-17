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
#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vitis/ai/reid.hpp>
#include "calibration.hpp"
#include "structure.hpp"

namespace vitis {
namespace ai {

using namespace std;
using namespace cv;

using Tracker = std::shared_ptr<FTD_Trajectory>;
using MotTracker = std::shared_ptr<FTD_Structure>;
using TrackerResult = pair<Rect2f, uint64_t>;

struct person_info {
  Mat reid_feat;
  vector<int> bbox;
  float score;
  int label;
  int oritenidx;
  int local_id;
};
struct cros_reid_info {
  uint64_t frame_id;
  vector<person_info> person_infos;  // Less than 4 people
};
void filter_detect_ROI(vector<person_info>& person_infos, Mat roi);

class CrossCameraTracker {
 public:
  CrossCameraTracker(vector<int> cams, vector<camInfo> camInfos, vector<MotTracker> motTrackers,
                     int speed_limit, float overlap_distance_tolerance);
  ~CrossCameraTracker();
  void updateSystem();
  void unit_id(Tracker tracker1, Tracker tracker2);
  void link_cross_tracker(Tracker tracker1, Tracker tracker2);
  void get_result();
  void gather_lostTrack();

  std::vector<Tracker> gather_activeTrack(int time_since_update = 5);
  vector<Tracker> find_crosscameraTrack(Tracker t);
  bool appear_same_cam(Tracker t1, Tracker tracker);
  bool appear_same_cam(Tracker t1, vector<Tracker> trackers);
  void merge_motTrackers();
  void remove_walkout_tracker();
  void update_print_info();
  void remove_tracker(int tids);
  void remove_lostTrack(int tids);
  bool spatialtemp_condition(Tracker t1, Tracker t2);
  void link_lost_and_new_track(vector<Tracker> disappear_trackers,
                               vector<Tracker> new_trackers, int my_case);
  void recover_tracker();
  vector<vector<TrackerResult>> result_;

 private:
  vector<int> cams_;
  vector<camInfo> camInfos_;
  std::vector<Tracker> tracks_;
  vector<MotTracker> motTrackers_;
  vector<Tracker> lostTrack_;
  float speed_limit_;
  float overlap_distance_tolerance_;
  int min_hits_ = 3;
  int frame_count_ = 0;
  float reid_thresh_ = 1.0;
};

}  // namespace ai
}  // namespace vitis
