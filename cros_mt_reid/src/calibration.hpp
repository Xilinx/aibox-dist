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

#include <glog/logging.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>

using namespace std;
using namespace cv;

struct camInfo {
  Mat image2worldTrans;
  Mat imageROI;
};
const int g_img_w = 2304;
const int g_img_h = 1296;
const int g_regionW = 1167;
const int g_regionH = 1250;

std::tuple<bool,vector<camInfo>> createCamInfo(std::string confFile, vector<int> cams,
                    int realw = 2304, int realh = 1296);
