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
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <ratio>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

#include <vitis/ai/classification.hpp>
#include <vitis/ai/profiling.hpp>
#include <vitis/ai/refinedet.hpp>
#include <vitis/ai/reid.hpp>
#include <vitis/ai/reidtracker.hpp>
#include "../src/mtmc_reid.hpp"

using namespace cv;
using namespace vitis::ai;
using namespace std;
using namespace vitis;
using namespace std::chrono;

//int img_w = 2304;
uint64_t imgCount = 0;
int idxInputImage = 1;  // image index of input video
int idxShowImage = 1;   // next frame index to be display
bool bReading = true;   // flag of input
int flag = 0;

// class cros_paircomp {
// public:
//  bool operator()(const cros_reid_info &n1, const cros_reid_info &n2) const {
//    if (n1.frame_id == n2.frame_id) return n1.frame_id > n2.frame_id;jj
//    return n1.frame_id > n2.frame_id;
//  }
//};

vector<int> cams{0, 1, 2};
//vector<int> cams{0, 1};
Rect2f int2rect(vector<int> bbox) {
  return Rect2f(bbox[0], bbox[1], bbox[2], bbox[3]);
}

mutex mtxQueueInput;  // mutex of input queue
mutex mtxQueueShow;   // mutex of display queue
// priority_queue<cros_reid_info, vector<cros_reid_info>, cros_paircomp>
//    queueShow;  // display queue

/* Base parameter and image path.*/
string baseImagePath;
string g_outfile;
vector<string> images;
vector<vector<Mat>> read_imgs;
queue<vector<cros_reid_info>> queueInput;  // input queue
// vector<vector<cros_reid_info>> input_infos;
const vector<string> cam_img_paths{"/img1/", "/img2/", "/img3/"};
void reader() {
  auto reid = vitis::ai::Reid::create("personreid-res18_pt");
  auto orientation = vitis::ai::Classification::create("orientation");
  auto det = vitis::ai::RefineDet::create("refinedet_pruned_0_92");
  //auto det = vitis::ai::RefineDet::create("refinedet_pruned_0_96");

  std::vector<InputCharact> input_characts;
  // string outfile = baseImagePath.substr(0, baseImagePath.size() - 1) +
  // ".txt";
  // input_infos.resize(cams.size());
  vector<cros_reid_info> cams_input_info;
  for (size_t i = 0; i < images.size(); ++i) {
    for (auto cam : cams) {
      string imageName = images[i];
      auto total = baseImagePath + cam_img_paths[cam] + imageName;
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << total;
      Mat image = imread(baseImagePath + cam_img_paths[cam] + imageName);
      // resize(image, image, Size(480, 360));
      auto results = det->run(image);
      int lid = 0;
      vector<person_info> pinfos;
      for (auto box : results.bboxes) {
        Rect in_box = Rect(box.x * image.cols, box.y * image.rows,
                           box.width * image.cols, box.height * image.rows);
        auto roi_range = cv::Rect(0, 0, image.cols, image.rows);
        in_box &= roi_range;
        vector<int> out_box = {in_box.x, in_box.y, in_box.width, in_box.height};
        Mat img = image(in_box);
        auto feat = reid->run(img).feat;
        auto ori = orientation->run(img).scores[0].index;
        person_info pinfo = {feat, out_box, box.score, -1, ori, lid};
        pinfos.emplace_back(pinfo);
      }
      cros_reid_info crinfo = {uint64_t(i+1), pinfos};
      cams_input_info.push_back(crinfo);
      LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER))
          << cam << " " << i << " " << cams_input_info.size();
      // input_infos[cam].push_back(crinfo);
    }
    CHECK(cams_input_info.size() == cams.size())
        << " error size: " << cams_input_info.size() << " vs " << cams.size();
    LOG_IF(INFO, ENV_PARAM(DEBUG_REID_TRACKER)) << queueInput.size();
    if (queueInput.size() < 30) {
      queueInput.push(cams_input_info);
      cams_input_info.clear();
    }
    while (queueInput.size() >= 30) usleep(1000);
  }

  cout << "##############################video end, count: " << imgCount + 0
       << endl;
  usleep(2000);
}

string num2str(int i) {
  char ss[10];
  sprintf(ss, "%04d", i);
  return ss;
}

void run() {
  float speed = 100 / 30;
  int overlap_distance_tolerance = 100;
  vector<MotTracker> motTrackers(cams.size());
  auto camInfos = createCamInfo("camsetup.json", cams);
  for (auto cam : cams) {
    auto camInfo = camInfos[cam];
    motTrackers[cam] =
        make_shared<FTD_Structure>(camInfo.image2worldTrans, cam);
  }
  auto system = make_shared<CrossCameraTracker>(
      cams, camInfos, motTrackers, speed, overlap_distance_tolerance);
  ofstream of(g_outfile);
  chrono::system_clock::time_point start_time = chrono::system_clock::now();

  std::vector<InputCharact> input_characts;
  vector<cros_reid_info> cams_input_info;
  uint64_t frame_id = 0u;
  int count = 0;
  while (1) {
    if (queueInput.empty()) {
      usleep(1000);
      continue;
    } else {
      cams_input_info = queueInput.front();
      queueInput.pop();
    }
    __TIC__(total);
    for (auto cam : cams) {
      auto input_info = cams_input_info[cam];
      frame_id = input_info.frame_id;
      input_characts.clear();
      auto person_infos = input_info.person_infos;
      LOG(INFO)<<person_infos.size(); 
      filter_detect_ROI(person_infos,camInfos[cam].imageROI.clone());
      string name = "filroi" + to_string(cam)+".jpg";
      imwrite(name, camInfos[cam].imageROI);
      LOG(INFO)<<person_infos.size(); 
      for (auto p_info : person_infos) {
        InputCharact ic = make_tuple(p_info.reid_feat, p_info.oritenidx,
                                     int2rect(p_info.bbox), p_info.score,
                                     p_info.label, p_info.local_id);
       // if(!det_inside_ROI(int2rect(p_info.bbox, camInfos[cam].imageROI))) continue;
        input_characts.emplace_back(ic);
      }
      __TIC__(update);
      motTrackers[cam]->Update(frame_id, input_characts);
      __TOC__(update);
    }
    __TIC__(system);
    system->updateSystem();
    LOG(INFO) << "frame id : " << frame_id + 0 << " count: "<<count<<" "<<int(count*24.98f) + 1<<endl;
    if(int(frame_id) == int(count*24.98f) + 1){
      for (int idx = 0; idx < int(cams.size());++idx) {
        auto cam = cams[idx];
        for (auto res : system->result_[cam]) {
          auto det = res.first;
          of<<count+1<<","<<res.second<<","<<det.x+g_img_w*idx<<","<<det.y<<","<<det.width<<","<<det.height<<endl;
          LOG(INFO) << "frame id : " << frame_id + 0 << " cam: " << cam
                    << " tid: " << res.second << " det: " << res.first;
        }
      }
      count+=1;
    }
    
    __TOC__(system);
    __TOC__(total);
    if ((frame_id + 1) == imgCount) break;
  }
  //writer.release();
  of.close();
  chrono::system_clock::time_point end_time = chrono::system_clock::now();
  auto dura = (duration_cast<microseconds>(end_time - start_time)).count();
  stringstream buffer;
  buffer << fixed << setprecision(2) << (float)imgCount / (dura / 1000000.f);
  string a = buffer.str() + "FPS";
  LOG(INFO) << " FPS : " << a;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    cout << "Please set input image and output file name. " << endl;
    return 0;
  }
  baseImagePath = string(argv[1]);
  g_outfile = string(argv[2]);
  std::ifstream fs(baseImagePath + "/list.txt");
  std::string line;
  while (getline(fs, line)) {
    images.emplace_back(line);
  }
  imgCount = images.size();
  if (images.size() == 0) {
    cerr << "\nError: Not images exist in " << baseImagePath << endl;
    return -1;
  }
  fs.close();
  array<thread, 2> threads = {thread(reader), thread(run)};
  for (int i = 0; i < 2; i++) {
    threads[i].join();
  }
  exit(0);
  return 0;
}
