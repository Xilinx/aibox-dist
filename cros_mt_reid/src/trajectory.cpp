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

#include "trajectory.hpp"
#include <glog/logging.h>
#include <iostream>
#include <thread>
#include <map>

namespace vitis {
namespace ai {

FTD_Trajectory::FTD_Trajectory() {}

void FTD_Trajectory::Predict() {
  age += 1;
  if (time_since_update > 0) hit_streak = 0;
  time_since_update += 1;
  std::get<2>(charact) = filter.GetPre();
}

int FTD_Trajectory::GetId() { return id; }

void FTD_Trajectory::SetId(uint64_t& update_id) { id = update_id; }

InputCharact& FTD_Trajectory::GetCharact() { return charact; }
int FTD_Trajectory::getAge() {
  return dets[dets.size() - 1].first - dets[0].first;
}
int FTD_Trajectory::getHits() { return dets.size(); }
frame_rect FTD_Trajectory::getDets() { return dets; }

vector<vector<float>> Mat2vect(const Mat& img) {
  CHECK(img.size() == Size(3, 3)) << "size should be 3x3: " << img.size();
  vector<vector<float>> vec2ret(3, vector<float>(3, 0.0f));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      vec2ret[i][j] = img.at<double>(i, j);
    }
  }
  return vec2ret;
}

Point2f transformPoint(const Mat& img, Point2f point) {
  CHECK(img.size() == Size(3, 3)) << "size should be 3x3: " << img.size();
  vector<vector<float>> transform = Mat2vect(img);
  vector<float> arry_point = {point.x, point.y, 1.0f};
  CHECK(transform.size() == arry_point.size()) << "";
  vector<float> temp(transform.size(), 0);
  for (auto i = 0u; i < transform.size(); ++i) {
    float sum = 0.0f;
    for (auto j = 0u; j < transform[i].size(); ++j) {
      // cout<<"trans: "<<transform[i][j]<<" " <<arry_point[j]<<" " <<
      // (transform[i][j] * arry_point[j])<<endl;
      sum += (transform[i][j] * arry_point[j]);
      // cout<<"sum: "<<sum<<endl;
    }
    temp[i] = sum;
    // cout<<"temp[i]: "<<temp[i]<<endl;
  }
  // LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
  //    << img << endl
  //    << point << Point2f{temp[0] / temp[2], temp[1] / temp[2]} << " "
  //    << temp[0] << " " << temp[1] << " " << temp[2] << endl;
  return Point2f{temp[0] / temp[2], temp[1] / temp[2]};
};
Point2f feet_pos(Rect_<float> det) {
  // LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<det<<" "<<Point2f(det.x +
  // det.width / 2, det.y + det.height);
  return Point2f(det.x + det.width / 2, det.y + det.height);
}
Point2f FTD_Trajectory::to_world(Point2f point) {
  // LOG_IF(INFO,
  // ENV_PARAM(DEBUG_REID_TRACKER))<<point<<transformPoint(transformMat, point);
  return transformPoint(transformMat, point);
}
Point2f FTD_Trajectory::get_last_det_world() {
  return to_world(feet_pos((*dets.end()).second));
}
Point2f FTD_Trajectory::get_first_det_world() {
  return to_world(feet_pos((*dets.begin()).second));
}
Rect_<float> FTD_Trajectory::get_state() { return filter.GetPost(); }
Point2f FTD_Trajectory::get_current_feet_worldpos() {
  return to_world(feet_pos(get_state()));
}
static uint64_t g_count_id = 0u;
void FTD_Trajectory::Init(Mat transImg, const InputCharact& input_charact,
                          uint64_t frame, int in_cam) {
  id = g_count_id;
  g_count_id++;
  // std::cout<<"new id: "<<id<<endl;
  cam = in_cam;
  transformMat = transImg;
  ori_feature.resize(4);
  charact = input_charact;
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
      << "Init a new trajectory(id " << id << ", bbox " << std::get<2>(charact)
      << ", label " << std::get<3>(charact) << ")";
  // Init FTD_ReidTracker and FTD_Filter
  filter.Init(std::get<2>(charact));
  DetectsAdd(frame, std::get<2>(charact));
}

struct lex_compare {
    bool operator() (const weak_ptr<FTD_Trajectory> &lhs, const weak_ptr<FTD_Trajectory> &rhs)const {
        auto lptr = lhs.lock(), rptr = rhs.lock();
        if (!rptr) return false; // nothing after expired pointer 
        if (!lptr) return true;  // every not expired after expired pointer
        return lptr.get() < rptr.get();
    }
};

class SameTrackerBook
{
  public:
    static SameTrackerBook & GetSameTrackerBook() { static SameTrackerBook book;
      return book;
    }

    void Update(uint64_t id, std::weak_ptr<FTD_Trajectory> wptr) {
      std::lock_guard<std::mutex> lock(mtx);

      if (auto sptr = wptr.lock()) {
        if (camMap.find(sptr->cam) == camMap.end()) {
          camMap[sptr->cam] = std::map<uint64_t, std::set<std::weak_ptr<FTD_Trajectory>, lex_compare>>();
        }
        auto& map = camMap[sptr->cam];

        if (map.size() > 30000) {
          map.erase(map.begin());
        }

        if ( map.find(id) == map.end() )
        {
          map[id] = std::set<std::weak_ptr<FTD_Trajectory>, lex_compare>({wptr});
        }
        else
        {
          // tell others
          for (auto wp : map[id]) {
            auto sp = wp.lock() ;
            if (sp && sptr)
            {
              sp->BookSameTracker(sptr.get());
              sptr->BookSameTracker(sp.get());
            }
          }
          map[id].insert(wptr);
        }
      }
    }

  private:
    SameTrackerBook() {};
    SameTrackerBook(SameTrackerBook const&)               = delete;
    void operator=(SameTrackerBook const&)  = delete;
    std::map<int, std::map<uint64_t, std::set<std::weak_ptr<FTD_Trajectory>, lex_compare>>> camMap;
    std::map<uint64_t, std::set<std::weak_ptr<FTD_Trajectory>, lex_compare>> map;
    std::mutex mtx;
};

FTD_Trajectory::~FTD_Trajectory()
{
}


void FTD_Trajectory::BookSameTracker(FTD_Trajectory* ptr)
{
  sameTrackers.insert(ptr);
}

void FTD_Trajectory::DetectsAdd(uint64_t frame, Rect_<float>&val)
{
  dets.emplace_back(frame, val);

  SameTrackerBook::GetSameTrackerBook().Update(frame, weak_from_this());
}

bool FTD_Trajectory::SameAs(FTD_Trajectory& other) 
{
  bool check = &other == this || sameTrackers.find(&other) != sameTrackers.end();
  return check;
}

void FTD_Trajectory::UpdateDetect(const InputCharact& input_charact,
                                  uint64_t frame) {
  // Update charact
  CHECK(std::get<4>(charact) == std::get<4>(input_charact))
      << "UpdateDetect must have the same label";
  charact = input_charact;
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
      << "trajectory " << id << " update_detector with bbox "
      << std::get<2>(charact) << " detsize: " << dets.size();
  time_since_update = 0;
  age += 1;
  hit_streak += 1;
  // Init FTD_ReidTracker and Update FTD_Filter
  filter.UpdateDetect(std::get<2>(charact));
  DetectsAdd(frame, std::get<2>(charact));
}

void FTD_Trajectory::UpdateTrack() {
  // update FTD_ReidTracker and Update FTD_Filter
  filter.UpdateFilter();
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
      << "trajectory " << id << " update_filter";
  std::get<5>(charact) = -1;
}

void FTD_Trajectory::UpdateFeature(const cv::Mat& feat, int oridx = -1) {
  if (feature.size() == cv::Size(0, 0))
    feature = feat;
  else
    feature = feature * 0.9 + feat * 0.1;
  if (features.size() > 1) features.pop_back();
  features.emplace_back(feature);
  if (oridx > -1) {
    if (ori_feature[oridx].size() == cv::Size(0, 0)) {
      ori_feature[oridx] = feat;
    } else
      ori_feature[oridx] = ori_feature[oridx] * 0.9 + feat * 0.1;
  }
}
std::vector<cv::Mat> FTD_Trajectory::GetFeatures() { return features; }
std::vector<cv::Mat> FTD_Trajectory::GetFeatures(int oridx) {
  if (age < 30 || oridx == -1) {
    return features;
  } else {
    auto feat = ori_feature[oridx];
    auto ret_feat = ori_feature;
    if (feat.size() != Size(0, 0))
      ret_feat[0] = feat;
    else
      ret_feat[0] = features[0];
    return ret_feat;
  }
  return features;
}
void FTD_Trajectory::UpdateWithoutDetect() {
  // update FTD_ReidTracker and Update FTD_Filter
  filter.UpdateFilter();
  std::get<5>(charact) = -1;
}

OutputCharact FTD_Trajectory::GetOut() {
  return std::make_tuple(id, filter.GetPost(), std::get<3>(charact),
                         std::get<4>(charact), std::get<5>(charact));
}

}  // namespace ai
}  // namespace vitis
