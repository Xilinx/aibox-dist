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

#include <jansson.h>
#include <iostream>
#include <algorithm>
#include <glog/logging.h>
#include "parse_json_config.hpp"

bool ParseFloorPlan(json_t *karray, CalibrationConfig &config)
{
  if (!json_is_array (karray)) {
    LOG(ERROR) << "Calibration JSON Error: FloorPlan is not array";
    return false;
  }
  uint pcount = json_array_size (karray);
  config.plan_pts.resize(pcount);
  int max_x = 0, max_y = 0;
  for (uint i = 0; i < pcount; i++ ) {
    json_t* obj = json_array_get (karray, i);
    int x = json_integer_value(json_object_get(obj, "x"));
    int y = json_integer_value(json_object_get(obj, "y"));
    config.plan_pts[i] = cv::Point(x, y);
    max_x = std::max(x, max_x);
    max_y = std::max(y, max_y);
  }
  config.plan_w = max_x;
  config.plan_h = max_y;
  return true;
}

bool ParseCams(float ratiow, float ratioh, json_t *karray, CalibrationConfig &config)
{
  if (!json_is_array (karray)) {
       LOG(ERROR) << "Calibration JSON Error: Cams is not array";
       return false;
  }
  uint camCount= json_array_size (karray);
  config.imagePoints.resize(camCount);
  config.worldPoints.resize(camCount);

  for (uint i = 0; i < camCount; i++) {
    json_t* cam_points = json_array_get (karray, i);

    uint camPointCount = json_array_size (cam_points);
    config.imagePoints[i].resize(camPointCount);
    config.worldPoints[i].resize(camPointCount);

    if (camPointCount != 4) {
       LOG(ERROR) << "Error: Number of calibration points for camera " << i << " is not 4.\n";
       return false;
    }
    for (uint j = 0; j < camPointCount; j++) {
      json_t* point = json_array_get (cam_points, j);

      json_t* img = json_object_get(point, "image");
      json_t* plan= json_object_get(point, "plan");

      config.imagePoints[i][j].x = json_integer_value(json_object_get(img, "x")) * ratiow;
      config.imagePoints[i][j].y = json_integer_value(json_object_get(img, "y")) * ratioh;

      config.worldPoints[i][j].x = json_integer_value(json_object_get(plan, "x"));
      config.worldPoints[i][j].y = json_integer_value(json_object_get(plan, "y"));
    }
  }
  return true;
}

bool ParseCalibrationConifig(std::string file, int realw, int realh, CalibrationConfig &config)
{
  json_t *root = NULL, *value;
  json_error_t error;

  root = json_load_file (file.c_str(), JSON_DECODE_ANY, &error);

  value = json_object_get (root, "floorplan");
  bool ret = ParseFloorPlan(value, config);

  value = json_object_get (root, "camsres");
  config.img_w = json_integer_value(json_object_get(value, "w"));
  config.img_h = json_integer_value(json_object_get(value, "h"));

  value = json_object_get (root, "cams");
  ret = ret && ParseCams((float)realw / config.img_w, (float)realh / config.img_h, value, config);

  json_decref(root);
  return ret;
}
