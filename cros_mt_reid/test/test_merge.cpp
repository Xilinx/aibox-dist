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
#include "../../src/cross_track.hpp"

using namespace cv;
using namespace vitis::ai;
using namespace std;
using namespace vitis;
using namespace std::chrono;

uint64_t imgCount = 0;
int idxInputImage = 1;  // image index of input video
int idxShowImage = 1;   // next frame index to be display
bool bReading = true;   // flag of input
int flag = 0;
int savepic = 0;

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
vector<string> images;
vector<vector<Mat>> read_imgs;
queue<vector<cros_reid_info>> queueInput;  // input queue
// vector<vector<cros_reid_info>> input_infos;
const vector<string> cam_img_paths{"/img1/", "/img2/", "/img3/"};
void reader() {
  auto reid = vitis::ai::Reid::create(                 "/opt/xilinx/kv260-aibox-dist/share/vitis_ai_library/models/personreid-res18_pt/personreid-res18_pt.xmodel");
  auto orientation = vitis::ai::Classification::create("/opt/xilinx/kv260-aibox-dist/share/vitis_ai_library/models/person-orientation_pruned_558m_pt/person-orientation_pruned_558m_pt.xmodel");
  auto det = vitis::ai::RefineDet::create(             "/opt/xilinx/kv260-aibox-dist/share/vitis_ai_library/models/refinedet_pruned_0_92/refinedet_pruned_0_92.xmodel");

  printf("reid fix point: %d\n", reid->get_input_fix_point());
  printf("ori  fix point: %d\n", orientation->get_input_fix_point());
  printf("refinedet fix point: %d\n", det->get_input_fix_point());

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
      int oriCols = image.cols, oriRows = image.rows;
      resize(image, image, Size(480, 360));
      auto results = det->run(image);
      int lid = 0;
      int limit = 0;
      vector<person_info> pinfos;
      printf("ROI CAM %d\n", cam);
      for (auto box : results.bboxes) {
        limit++;
        Rect in_box = Rect(box.x * image.cols, box.y * image.rows,
                           box.width * image.cols, box.height * image.rows);
        printf("ROI CAM %d: %d, %d, %d, %d\n", cam, in_box.x, in_box.y, in_box.width, in_box.height);

        auto roi_range = cv::Rect(0, 0, image.cols, image.rows);
        in_box &= roi_range;
        vector<int> out_box = {in_box.x, in_box.y, in_box.width, in_box.height};
        Mat img = image(in_box);
        auto feat = reid->run(img).feat;
        auto ori = orientation->run(img).scores[0].index;

        vector<int> ori_out_box = { box.x * oriCols, box.y * oriRows,
                           box.width * oriCols, box.height * oriRows};
        person_info pinfo = {feat, ori_out_box, box.score, -1, ori, lid};
        pinfos.emplace_back(pinfo);
      }
      printf("ROI CAM %d end\n", cam);

      cros_reid_info crinfo = {uint64_t(i), pinfos};
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
Mat convertTo3Channels(const Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
    vector<Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);
    return three_channel;
}

void run() {
  Mat image = imread(baseImagePath + cam_img_paths[0] + images[0]);
  int imgw = image.cols, imgh = image.rows;

  auto tracker = new CrossTracker("/opt/xilinx/kv260-aibox-dist/share/vvas/cam_setup.json", cams.size(), imgw, imgh);
 // ofstream of(outfile);
  chrono::system_clock::time_point start_time = chrono::system_clock::now();

  std::vector<InputCharact> input_characts;
  vector<cros_reid_info> cams_input_info;
  uint64_t frame_id = 0u;
  static int i = -1;
  VideoWriter writer;
  writer.open("cros_merge_results.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 15,
              Size(960, 540*cams.size()), true);
  while (1) {
    if (queueInput.empty()) {
      usleep(1000);
      continue;
    } else {
      cams_input_info = queueInput.front();
      queueInput.pop();
    }
    i++;
    __TIC__(total);
    vector<Mat> out_imgs(cams.size());
    for (auto cam : cams) {
      auto input_info = cams_input_info[cam];
      frame_id = input_info.frame_id;
      string imageName = images[frame_id];
      auto total = baseImagePath + cam_img_paths[cam] + imageName;
      out_imgs[cam] = imread(baseImagePath + cam_img_paths[cam] + imageName);
    }

    auto results = tracker->Run(cams_input_info);

    __TIC__(system);
    for (auto cam : cams) {
      auto image = out_imgs[cam];
      cv::putText(image, to_string(frame_id), cv::Point(10, 1000), 1, 3,
                  cv::Scalar{240, 240, 240}, 2);
      for (auto res : results[cam]) {
        LOG(INFO) << "frame id : " << frame_id + 0 << " cam: " << cam
                  << " tid: " << res.second << " det: " << res.first;
        cv::rectangle(image, res.first, cv::Scalar(255, 0, 0), 2, 1, 0);
        cv::putText(image, to_string(res.second), res.first.tl(), 1, 5,
                    cv::Scalar{0, 0, 255}, 2);
      }
      //string name = "roi" + to_string(cam)+".jpg";
      //imwrite(name, camInfos[cam].imageROI);
      resize(image,image,Size(960,540));
      Mat back_img, res_back_img;
      //resize(camInfos[cam].imageROI, res_back_img,Size(960,540) );
      //name = "res_roi" + to_string(cam)+".jpg";
      //imwrite(name, res_back_img);
      //back_img = convertTo3Channels(res_back_img);
      //name = "bac_img" + to_string(cam)+".jpg";
      //imwrite(name, back_img);
      out_imgs[cam] = image;// + 0.4 * back_img;
      //name = "out_img" + to_string(cam)+".jpg";
      //imwrite(name, out_imgs[cam]);
    }

    Mat show_img;
    vconcat(out_imgs[0], out_imgs[1], show_img);
    for (size_t i = 2; i < cams.size(); i++) {
        vconcat(show_img, out_imgs[i], show_img);
    }
    LOG(INFO)<<show_img.size();
    writer.write(show_img);
    __TOC__(system);
    __TOC__(total);
    if (savepic) {
      for (auto cam : cams) {
        string imageName = images[i];
        auto total = baseImagePath + cam_img_paths[cam] + imageName;
        Mat image = imread(total);

        float rw = 1, rh = 1;
        //float rw = image.cols / 480.0, rh = image.rows / 360.0;
        for (auto res : results[cam]) {
          LOG(INFO) << "frame id : " << i << " cam: " << cam
                    << " tid: " << res.second << " det: " << res.first << ";"
                    << image.cols << "," << image.rows << ";"
                    << rw << "," << rh << ";"
                    << rw*res.first.x << "," << rh*res.first.y << ";"
                    << rw*(res.first.x+res.first.width) << "," << rh*(res.first.y+res.first.height) 
                    ;
              ostringstream oss;
              oss << res.second;
              rectangle(image, Point(rw*res.first.x, rh*res.first.y),
                      Point(rw*(res.first.x+res.first.width), rh*(res.first.y+res.first.height)),
                      Scalar(255,0,0), 3, 1, 0);
              putText(image, oss.str().c_str(), Point(rw*res.first.x, rh*res.first.y), 3, 3,
                      Scalar(0,0,255), 3, 1);
        }
        imwrite(baseImagePath + cam_img_paths[cam] + "result_" + imageName, image);
        LOG(INFO) << "write result:" << (baseImagePath + cam_img_paths[cam] + "result_" + imageName) << "," << image.cols << "," << image.rows << endl;
      }
    }
    if ((frame_id + 1) == imgCount) break;
  }
  //writer.release();
  chrono::system_clock::time_point end_time = chrono::system_clock::now();
  auto dura = (duration_cast<microseconds>(end_time - start_time)).count();
  stringstream buffer;
  buffer << fixed << setprecision(2) << (float)imgCount / (dura / 1000000.f);
  string a = buffer.str() + "FPS";
  LOG(INFO) << " FPS : " << a;
}

int main(int argc, char **argv) {
  // google::InitGoogleLogging(argv[0]);
  if (argc < 2) {
    cout << "Please set input image. " << endl;
    return 0;
  }
  savepic = argc > 2;
  baseImagePath = string(argv[1]);
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
