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

#include "calibration.hpp"
#include "parse_json_config.hpp"

Mat getROImask(vector<Point> pts, float ratiow, float ratioh, const CalibrationConfig& config) {
  int regionW = config.plan_w * ratiow;
  int regionH = config.plan_h * ratioh;
  Mat mat_ret = Mat(regionH, regionW, CV_8UC1);
  fillPoly(mat_ret, pts, Scalar(255));
  //imwrite("plan.jpg", mat_ret);
  return mat_ret;
}

void TestImage(const Mat& in, vector<Point2f>& ip, vector<Point2f>& wp, std::string name, int ind)
{
  ostringstream oss;
  Mat tmp = in.clone();
  oss << name << "-" << ind << ".jpg";

  for (auto p : ip) {
    cv::circle(tmp, p, 3, Scalar(0, 0, 255), 2);
  }
  for (auto p : wp) {
    cv::circle(tmp, p, 3, Scalar(0, 255, 0), 2);
  }
  cv::imwrite(oss.str().c_str(), tmp);
}

std::tuple<bool,vector<camInfo>> createCamInfo(std::string confFile, vector<int> cams, int realw, int realh) {
  vector<camInfo> rets(cams.size());
  CalibrationConfig config;
  if ( !ParseCalibrationConifig(confFile, realw, realh, config) )
      return std::make_tuple(false, rets);

  int img_w = config.img_w;
  int img_h = config.img_h;
  float ratiow = (float)realw / img_w;
  float ratioh = (float)realh / img_h;
  vector<Point> &plan_pts = config.plan_pts;
  vector<vector<Point2f>> &imagePoints = config.imagePoints;
  vector<vector<Point2f>> &worldPoints = config.worldPoints;
  auto plan = getROImask(plan_pts, ratiow, ratioh, config);
  Mat world2imageTrans, image2worldTrans, imageROI;
  for (auto i = 0u; i < cams.size(); ++i) {
    auto cam = cams[i];
    auto pts1 = worldPoints[cam];
    auto pts2 = imagePoints[cam];
    world2imageTrans = getPerspectiveTransform(pts1, pts2);
    image2worldTrans = getPerspectiveTransform(pts2, pts1);
    warpPerspective(plan, imageROI, world2imageTrans, Size(img_w, img_h));
    rets[cam].image2worldTrans = image2worldTrans;
    rets[cam].imageROI = imageROI.clone();
  }
  return std::make_tuple(true, rets);
}
