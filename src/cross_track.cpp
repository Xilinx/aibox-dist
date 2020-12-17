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

#include <zconf.h>
#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <ratio>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>
#include <vitis/ai/profiling.hpp>
#include "cross_track.hpp"

using namespace cv;
using namespace vitis::ai;
using namespace std;
using namespace vitis;
using namespace std::chrono;

static Rect2f int2rect(vector<int> bbox) {
//  LOG(INFO)<<bbox[0]<<bbox[1]<<bbox[2]<<bbox[3];
  return Rect2f(bbox[0], bbox[1], bbox[2], bbox[3]);
}

CrossTracker::CrossTracker(std::string confFile, int num, int w, int h)
    :constrOK(false)
{
  cams.resize(num);
  for (int i = 0; i < num; i++) {
    cams[i] = i;
  }
  float speed = 100 / 30;
  int overlap_distance_tolerance = 100;
  motTrackers.resize(cams.size());
  std::tie(constrOK, camInfos) = createCamInfo(confFile, cams, w, h);
  for (auto cam : cams) {
    auto camInfo = camInfos[cam];
    motTrackers[cam] =
        make_shared<FTD_Structure>(camInfo.image2worldTrans, cam);
  }
  trackSys = make_shared<CrossCameraTracker>(cams, camInfos, motTrackers, speed,
                                                overlap_distance_tolerance);
}

vector<vector<TrackerResult>>
CrossTracker::Run(vector<cros_reid_info*>& input_infos) {
  __TIC__(total);
  for (auto cam : cams) {
    auto input_info = input_infos[cam];
    auto frame_id = input_info->frame_id;

    auto person_infos = input_info->person_infos;
//    filter_detect_ROI(person_infos, camInfos[cam].imageROI.clone());

    std::vector<InputCharact> input_characts;
    input_characts.clear();
    for (auto p_info : input_info->person_infos) {
      InputCharact ic = make_tuple(p_info.reid_feat, p_info.oritenidx,
                                   int2rect(p_info.bbox), p_info.score,
                                   p_info.label, p_info.local_id);
      input_characts.emplace_back(ic);
    }
    __TIC__(update);
    motTrackers[cam]->Update(frame_id, input_characts);
    __TOC__(update);
  }
  __TIC__(system);
  trackSys->updateSystem();
  __TOC__(system);
  __TOC__(total);

  return trackSys->result_;
}

vector<vector<TrackerResult>>
CrossTracker::Run(vector<cros_reid_info>& input_infos) {
  vector<cros_reid_info*> tmp;
  for (auto& i : input_infos) {
    tmp.push_back(&i);
  }
  return Run(tmp);
}
