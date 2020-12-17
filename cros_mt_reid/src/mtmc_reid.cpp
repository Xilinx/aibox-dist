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


#include "mtmc_reid.hpp"
#include <math.h>
#include <vitis/ai/profiling.hpp>

namespace vitis {
namespace ai {

double point_distance(Point2f pt1, Point2f pt2) {
  //LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "point distance: "<<pt1<< pt2<<" dis: "<< (pt1.x - pt2.x) * (pt1.x - pt2.x) +
  //            (pt1.y - pt2.y) * (pt1.y - pt2.y);
  return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) +
              (pt1.y - pt2.y) * (pt1.y - pt2.y));
}
double x_point_distance(Point2f pt1, Point2f pt2) {
  //LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "point distance: "<<pt1<< pt2<<" dis: "<< (pt1.x - pt2.x) * (pt1.x - pt2.x) +
  //            (pt1.y - pt2.y) * (pt1.y - pt2.y);
  return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x)
              );
}

bool is_inside_roi(Point pt, Mat roi) {
  //LOG(INFO)<<"is inside: "<<pt<<" w: "<<roi.cols<<" h: "<<roi.rows;
  if (pt.x >= roi.cols || pt.y >= roi.rows) return false;
  uint8_t val = uint8_t(roi.at<char>(pt.y, pt.x));
  //LOG(INFO)<<val+0<<" "<<pt<<" w: "<<roi.cols<<" h: "<<roi.rows;
  if(val+0 > 0) return true;
  return false;
}

Point get_feet_position(Rect det) {
  auto tl = det.tl();
  auto width = det.width;
  auto height = det.height;
  tl.x = tl.x + (width / 2);
  tl.y = tl.y + height;
  return tl;
}

bool det_inside_roi(Rect det, Mat roi) {
  auto feetPos = get_feet_position(det);
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<feetPos;
  return is_inside_roi(feetPos, roi);
}

Rect bbox2rect(vector<int> bbox) {
  return Rect(bbox[0], bbox[1], bbox[2], bbox[3]);
}

void filter_detect_ROI(vector<person_info>& person_infos, Mat roi) {
  auto tsize = person_infos.size();
  for (auto i = int(tsize - 1); i >= 0; i--) {
    auto det = bbox2rect(person_infos[i].bbox);
    LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<det;
    if (!det_inside_roi(det, roi)) {
      person_infos.erase(person_infos.begin()+i);
    }
  }
}
CrossCameraTracker::CrossCameraTracker(
  vector<int> cams, vector<camInfo> camInfos,
                                       vector<MotTracker> motTrackers,
                                       int speed_limit,
                                       float overlap_distance_tolerance) {
  cams_ = cams;
  camInfos_ = camInfos;
  motTrackers_ = motTrackers;
  speed_limit_ = speed_limit;
  overlap_distance_tolerance_ = overlap_distance_tolerance;
  result_.resize(cams_.size());
}
CrossCameraTracker::~CrossCameraTracker() {}

void CrossCameraTracker::updateSystem() {
  frame_count_ += 1;
  __TIC__(remove);
  remove_walkout_tracker();
  __TOC__(remove);
  __TIC__(merge);
  merge_motTrackers();
  __TOC__(merge);
  __TIC__(recover);
  recover_tracker();
  __TOC__(recover);
  __TIC__(gather);
  gather_lostTrack();
  __TOC__(gather);
  __TIC__(result);
  get_result();
  __TOC__(result);
}

void CrossCameraTracker::unit_id(Tracker tracker1, Tracker tracker2) {
  int min_id = min(tracker1->id, tracker2->id);
  vector<Tracker> t1_ct = find_crosscameraTrack(tracker1);
  vector<Tracker> t2_ct = find_crosscameraTrack(tracker2);
  tracker1->id = min_id;
  tracker2->id = min_id;
  for (auto ct : t1_ct) {
    ct->id = min_id;
  }
  for (auto ct : t2_ct) {
    ct->id = min_id;
  }
}
void CrossCameraTracker::link_cross_tracker(Tracker t1, Tracker t2) {
  if (t1->id == t2->id || t1->cam == t2->cam) return;
  auto t1_ct = find_crosscameraTrack(t1);
  auto t2_ct = find_crosscameraTrack(t2);
  for (auto i = 0u; i < t1_ct.size(); ++i) {
    if (t2->cam == t1_ct[i]->cam) return;
  }
  for (auto i = 0u; i < t2_ct.size(); ++i) {
    if (t1->cam == t2_ct[i]->cam) return;
  }
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
      << " t1id " << t1->id << " t2id " << t2->id;
  unit_id(t1, t2);
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
      << " t1id " << t1->id << " t2id " << t2->id;
}
static uint64_t g_show_id = 0;
static map<uint64_t,uint64_t> map_ids;
void CrossCameraTracker::get_result() {
  for (auto cam : cams_) {
    result_[cam].clear();
    for (auto t : motTrackers_[cam]->tracks) {
      auto track_rect = t->get_state();
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<track_rect;
      if (t->time_since_update < 1) {
        auto tid = t->GetId();
        if(map_ids.find(tid) == map_ids.end()){
          map_ids[tid] = g_show_id;
          g_show_id++;
        }
          result_[cam].emplace_back(track_rect, map_ids[tid]);
      }
    }
  }
}
void CrossCameraTracker::gather_lostTrack() {
  for (auto cam : cams_) {
    auto trackers = motTrackers_[cam]->tracks;
    auto tsize = trackers.size();
    for (auto i = int(tsize - 1); i >= 0; --i) {
      auto t = trackers[i];
      if (t->time_since_update > 100) {
        auto t_ct = find_crosscameraTrack(t);
        CHECK(t_ct.size() > 0) << " none ct";
        for (auto ct : t_ct) {
          CHECK(ct->time_since_update <= 100)
              << " time update : " << ct->time_since_update;
        }
        lostTrack_.push_back(t);
        LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<t->time_since_update;
        trackers.erase(trackers.begin() + i);
      } else if (t->time_since_update > 30 && t->getHits() < 5) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<t->time_since_update<<" hit: "<<t->getHits();
        trackers.erase(trackers.begin() + i);
      }
    }
  }
}

std::vector<Tracker> CrossCameraTracker::find_crosscameraTrack(Tracker t1) {
  std::vector<Tracker> tracks;
  for (auto cam : cams_) {
    auto tsize = motTrackers_[cam]->tracks.size();
    for (auto i = int(tsize - 1); i >= 0; i--) {
      auto t = motTrackers_[cam]->tracks[i];
      //LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
      //    << "i: " << i << " id: " << t->id << " t1 id: " << t1->id;
      if (t->id == t1->id) {
        if (t->time_since_update > 100){
          LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<"erase";
          motTrackers_[cam]->tracks.erase(motTrackers_[cam]->tracks.begin() +
                                          i);
        }
        else{
          tracks.push_back(t);
        }
      }
    }
  }
  //LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << tracks.size();
  return tracks;
}

std::vector<Tracker> CrossCameraTracker::gather_activeTrack(
    int time_since_update) {
  std::vector<Tracker> tracks;
  for (auto cam : cams_) {
    for (auto t : motTrackers_[cam]->tracks) {
      if (t->time_since_update < time_since_update) tracks.push_back(t);
    }
  }
  return tracks;
}
bool CrossCameraTracker::appear_same_cam(Tracker t1, Tracker tracker) {
  __TIC__(MERGE_LOOP_ONE_APPEAR_SAME_SUB)
#if 0
  bool ocheck = false;
#endif
  bool check = false;
  if (t1->cam == tracker->cam) {
    auto det1 = t1->dets;
    auto det2 = tracker->dets;
    __TIC__(MERGE_LOOP_ONE_APPEAR_SAME_SUB_OPT)
    check = t1->SameAs(*tracker);
    __TOC__(MERGE_LOOP_ONE_APPEAR_SAME_SUB_OPT)
#if 0
    for(auto d1 : det1){
      for(auto d2 : det2){
        if(d1.first == d2.first) {
          ocheck = true;
          break;
        }
      }
    }
#endif
  }
  __TOC__(MERGE_LOOP_ONE_APPEAR_SAME_SUB)
  return check;
}
bool CrossCameraTracker::appear_same_cam(Tracker t1, vector<Tracker> trackers) {
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"appear_same_cam_vec: trackers size: "<< trackers.size() << endl;
  __TIC__(MERGE_LOOP_ONE_APPEAR_SAME_VEC_SUB)
  for (auto t : trackers) {
    if (appear_same_cam(t1, t)) {
      __TOC__(MERGE_LOOP_ONE_APPEAR_SAME_VEC_SUB)
      return true;
    }
  }
  __TOC__(MERGE_LOOP_ONE_APPEAR_SAME_VEC_SUB)
  return false;
}
struct Min_Pos_Result {
  int row;
  int col;
  double min;
};
Min_Pos_Result get_min_vec2(vector<vector<double>> matrix) {
  Min_Pos_Result min_pos_result{0, 0, 100.0};
  for (auto i = 0u; i < matrix.size(); ++i) {
    for (auto j = 0u; j < matrix[0].size(); ++j) {
      if (matrix[i][j] < min_pos_result.min) {
        min_pos_result.row = i;
        min_pos_result.col = j;
        min_pos_result.min = matrix[i][j];
      }
    }
  }
  return min_pos_result;
}

void CrossCameraTracker::merge_motTrackers() {
  auto trackers = gather_activeTrack();
  auto tsize = trackers.size();
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"size: "<<tsize<<endl;
  vector<vector<double>> world_distance_matrix(tsize, vector<double>(tsize, 0.0d));
  vector<vector<double>> reid_matrix(tsize, vector<double>(tsize, 100.0d));
  __TIC__(MERGE_LOOP)
  for (auto i = 0u; i < tsize; ++i) {
    for (auto j = 0u; j < i; ++j) {
      __TIC__(MERGE_LOOP_ONE)
      auto t1 = trackers[i];
      auto t2 = trackers[j];
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"cam1: "<<t1->cam<<" id1: "<<t1->id<<" cam2: "<<t2->cam<<" id2: "<<t2->id;
      if (t1->id == t2->id || t1->cam == t2->cam) {
          __TOC__(MERGE_LOOP_ONE)
          continue;
      }
      
      __TIC__(MERGE_LOOP_ONE_FIND)
      auto t1_ct = find_crosscameraTrack(t1);
      auto t2_ct = find_crosscameraTrack(t2);
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"appear same: "<< i << ", " << j << "," << tsize<< endl;
      __TOC__(MERGE_LOOP_ONE_FIND)
      __TIC__(MERGE_LOOP_ONE_APPEAR_SAME)
      if (appear_same_cam(t1, t2) || appear_same_cam(t1, t2_ct) ||
          appear_same_cam(t2, t1) || appear_same_cam(t2, t1_ct)) {
        __TOC__(MERGE_LOOP_ONE_APPEAR_SAME)
        __TOC__(MERGE_LOOP_ONE)
        continue;
      }
      __TOC__(MERGE_LOOP_ONE_APPEAR_SAME)
      //world_distance_matrix[i][j] = x_point_distance(
      __TIC__(MERGE_LOOP_ONE_OTHER)
      world_distance_matrix[i][j] = point_distance(
          t1->get_current_feet_worldpos(), t2->get_current_feet_worldpos());
      world_distance_matrix[j][i] = world_distance_matrix[i][j];
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"world dis : "<<world_distance_matrix[i][j]<<" thred: "<<overlap_distance_tolerance_;
      if (world_distance_matrix[i][j] < overlap_distance_tolerance_) {
        reid_matrix[i][j] = cosine_distance(t1->feature, t2->feature);
        reid_matrix[j][i] = reid_matrix[i][j];
      }
      __TOC__(MERGE_LOOP_ONE_OTHER)
      __TOC__(MERGE_LOOP_ONE)
    }
  }
  __TOC__(MERGE_LOOP)
  Min_Pos_Result ret = get_min_vec2(reid_matrix);
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"min ried: "<<ret.min;
  __TIC__(MERGE_WHILE)
  while (trackers.size() > 0 && ret.min < reid_thresh_) {
    for (auto i = 0u; i < reid_matrix.size(); ++i) {
      reid_matrix[i][ret.col] = 10.0d;
      reid_matrix[i][ret.row] = 10.0d;
    }
    for (auto i = 0u; i < reid_matrix[0].size(); ++i) {
      reid_matrix[ret.row][i] = 10.0d;
      reid_matrix[ret.col][i] = 10.0d;
    }
    auto mt1 = trackers[ret.row];
    auto mt2 = trackers[ret.col];
    LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"link: row: "<<ret.row<<" col: "<<ret.col;
    link_cross_tracker(mt1, mt2);
    ret = get_min_vec2(reid_matrix);
  }
  __TOC__(MERGE_WHILE)
}
void CrossCameraTracker::remove_walkout_tracker() {
  for (auto cam : cams_) {
    auto trackers = motTrackers_[cam]->tracks;
    auto tsize = trackers.size();
    LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << cam << " size: " << tsize;
    for (auto i = int(tsize - 1); i >= 0; --i) {
      auto t = trackers[i];
      auto t_crosscameraTracks = find_crosscameraTrack(t);
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << t_crosscameraTracks.size();
      int min_ct_tsu = 10000;
      for (auto ct : t_crosscameraTracks) {
        if (min_ct_tsu > ct->time_since_update){
           min_ct_tsu = ct->time_since_update;
           LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
              << "time us: " << ct->time_since_update<<" "<<ct->getHits()<<" "<<ct->GetId()<<" "<<min_ct_tsu;
        }
      }
      if (t_crosscameraTracks.size() && t->time_since_update > 50 &&
          min_ct_tsu < t->time_since_update) {
        LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<"erase size: "<<t_crosscameraTracks.size()
        <<" time: "<<t->time_since_update<<" min: "<<min_ct_tsu<<" tid: "<<t->GetId();
        trackers.erase(trackers.begin() + i);
      }
    }
    LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << cam << " size: " << tsize;
  }
}
void CrossCameraTracker::remove_tracker(int tids) {
  
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "remove tracker";
  for (int idx = 0; idx < int(cams_.size());++idx) {
    auto cam = cams_[idx];
    auto trackers = motTrackers_[cam]->tracks;
    auto tsize = trackers.size();
    for (int i = int(tsize-1); i >= 0; --i) {
      if (trackers[i]->id == tids) {
        trackers.erase(trackers.begin() + i);
        LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))<<"erase";
        break;
      }
    }
  }
}
void CrossCameraTracker::remove_lostTrack(int tids) {
  for (int i = int(lostTrack_.size()-1); i >= 0; --i) {
    if (lostTrack_[i]->id == tids) {
      lostTrack_.erase(lostTrack_.begin() + i);
      break;
    }
  }
}
float get_world_speed(Point2f pt1, Point2f pt2, uint64_t frames) {
  auto dis = point_distance(pt1, pt2);
  return dis / frames;
}
bool CrossCameraTracker::spatialtemp_condition(Tracker t1, Tracker t2) {
  auto start_frame = t2->dets[0].first;
  auto end_frame = t2->dets[t2->dets.size()-1].first;
  auto frames =  end_frame - start_frame ;
  if (frames < 1) return false;
  auto speed = get_world_speed(t1->get_last_det_world(),
                               t2->get_first_det_world(), frames);
                              
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "frame: "<<frames<<" speed: "<<speed<<" s: "<<start_frame<< " e: "<<end_frame;
  return speed < speed_limit_;
}

double cal_oriented_feature_dist(Tracker t1, Tracker t2) {
  vector<int> common_orientation;
  auto t1_ori_feats = t1->ori_feature;
  auto t2_ori_feats = t2->ori_feature;
  for (int idx = 0; idx < int(t1_ori_feats.size()); ++idx) {
    if (t1_ori_feats[idx].size() != Size(0, 0) &&
        t2_ori_feats[idx].size() != Size(0, 0))
      common_orientation.push_back(idx);
  }
  if (common_orientation.empty()) {
    return cosine_distance(t1->feature, t2->feature);
  } else {
    double min_dis = 2;
    for (auto idx : common_orientation) {
      auto cdis = cosine_distance(t1->ori_feature[idx], t2->ori_feature[idx]);
      min_dis = min(min_dis, cdis);
    }
    return min_dis;
  }
  return 1.0;
}

frame_rect update_frame_det(const frame_rect& t1, const frame_rect& t2){
  frame_rect ret;
  size_t i = 0, j = 0;
  while(i < t1.size() && j < t2.size()){
    if(t1[i].first < t2[j].first){
      ret.push_back(t1[i]);
      ++i;
    }
    else if(t1[i].first == t2[j].first){
      ret.push_back(t2[j]);
      ++i;
      ++j;
    }
    else{
      ret.push_back(t2[j]);
      ++j;
    }
  }
  return ret;
}

void CrossCameraTracker::link_lost_and_new_track(
    vector<Tracker> disappear_trackers, vector<Tracker> new_trackers,
    int my_case) {
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "link lost and new track";
  if (disappear_trackers.size() * new_trackers.size() == 0) return;
  vector<vector<double>> reid_matrix(
      disappear_trackers.size(), vector<double>(new_trackers.size(), 100.0));
  for (auto i = 0u; i < disappear_trackers.size(); ++i) {
    for (auto j = 0u; j < new_trackers.size(); ++j) {
      auto t1 = disappear_trackers[i];
      auto t2 = new_trackers[j];
      auto possible_link = spatialtemp_condition(t1, t2);
      auto t1_ct = find_crosscameraTrack(t1);
      auto t2_ct = find_crosscameraTrack(t2);
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) <<"appear same2: "<< i << "/ " << disappear_trackers.size() << ", " << j << "/" << new_trackers.size() << endl;
      __TIC__(MERGE_LOOP_ONE_APPEAR_SAME2)
      if (appear_same_cam(t1, t2) || appear_same_cam(t1, t2_ct) ||
          appear_same_cam(t2, t1) || appear_same_cam(t2, t1_ct)) {
        __TOC__(MERGE_LOOP_ONE_APPEAR_SAME2)
        continue;
      }
      __TOC__(MERGE_LOOP_ONE_APPEAR_SAME2)
      for (auto ct1 : t1_ct) {
        possible_link &= spatialtemp_condition(ct1, t2);
      }
      if (possible_link) reid_matrix[i][j] = cal_oriented_feature_dist(t1, t2);
    }
    Min_Pos_Result ret = get_min_vec2(reid_matrix);
    while (ret.min < reid_thresh_) {
      for (auto i = 0u; i < reid_matrix.size(); ++i) {
        reid_matrix[i][ret.col] = 10;
      }
      for (auto i = 0u; i < reid_matrix[0].size(); ++i) {
        reid_matrix[ret.row][i] = 10;
      }
      auto t1 = disappear_trackers[ret.row];
      auto t2 = new_trackers[ret.col];
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "det size: "<<t2->dets.size();
      t2->dets = update_frame_det(t1->dets, t2->dets); //need to do
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "det size: "<<t2->dets.size();
      if (my_case == 1) {
        remove_tracker(t1->id);
      } else {
        remove_lostTrack(t1->id);
      }
      t2->id = t1->id;
      vector<int> complement_orientation;
      auto t1_ori_feats = t1->ori_feature;
      auto t2_ori_feats = t2->ori_feature;
      for (int idx = 0; idx < 4; ++idx) {
        if (t1_ori_feats[idx].size() != Size(0, 0) &&
            t2_ori_feats[idx].size() != Size(0, 0))
          complement_orientation.push_back(idx);
      }
      for (auto co : complement_orientation) {
        t2->ori_feature[co] = t1->ori_feature[co];
      }
      ret = get_min_vec2(reid_matrix);
    }
  }
}

void CrossCameraTracker::recover_tracker() {
  LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << "recover tracker";
  vector<shared_ptr<FTD_Trajectory>> all_trackers;
  for (auto cam : cams_) {
    for (auto t : motTrackers_[cam]->tracks) {
      all_trackers.push_back(t);
    }
  }
  vector<shared_ptr<FTD_Trajectory>> active_tids;
  vector<shared_ptr<FTD_Trajectory>> disappear_trackers;
  vector<shared_ptr<FTD_Trajectory>> new_trackers;
  for (auto t : all_trackers) {
    if (t->time_since_update < 1) active_tids.push_back(t);
    if (t->getHits() < 30) new_trackers.push_back(t);
  }
  bool is_id_active = false;
  for (auto t : all_trackers) {
    if (t->time_since_update >= 1) {
      for (auto at : active_tids) {
        if (t->id == at->id) {
          is_id_active = true;
          break;
        }
      }
      if(!is_id_active) disappear_trackers.push_back(t);
      else is_id_active = false;
    }
  }
  link_lost_and_new_track(disappear_trackers, new_trackers, 1);
  link_lost_and_new_track(lostTrack_, new_trackers, 2);
}

}  // namespace ai
}  // namespace vitis
