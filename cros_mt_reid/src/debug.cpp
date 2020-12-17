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

#include <fstream>
#include <ostream>
#include "debug.hpp"

using namespace vitis::ai;

template<>
std::map<std::string, StreamCounter> ResultDumper<StreamCounter>::_ofs = {};

std::ostream &operator<<(std::ostream& oss, cros_reid_info const& info)
{
  oss << "Frame:" << info.frame_id << ", nump: " << info.person_infos.size() << "\n";
  int ip = 0;
  for (auto p : info.person_infos)
  {
    oss << "\tp: " << ip++ << "";
    oss << "\t\tbbox: [";
    for (auto box : p.bbox)
    {
      oss << box << ", ";
    }
    oss << "]; ";
    oss << " or: " << p.oritenidx << ";";
    oss << " li: " << p.local_id << ";";
    oss << " la: " << p.label << ";";
    oss << " sc: " << p.score << ";";
    oss << " fe: " << p.reid_feat << "\n";
  }
  return oss;
}

std::ostream &operator<<(std::ostream& oss, std::vector<cros_reid_info> const& cams_input_info)
{
    int icam = 0;
    for (const auto info : cams_input_info)
    {
      oss << "CAM " << icam++ << "; ";
      oss << info;
    }
    return oss;
}

std::ostream &operator<<(std::ostream& oss, std::vector<cros_reid_info*> const& cams_input_info)
{
    int icam = 0;
    for (const auto pinfo : cams_input_info)
    {
      oss << "CAM " << icam++ << "; ";
      oss << *pinfo;
    }
    return oss;
}

std::ostream &operator<<(std::ostream& oss, vector<vector<TrackerResult>> const& cams_input_info)
{
  int icam = 0;
  for (auto res : cams_input_info)
  {
    oss << "CAM " << icam++ << "; nump: " << res.size() << "\n";
    for (auto p : res)
    {
      oss << "\tid:" << p.second << "; ";
      oss << "rect:" << p.first << "\n";
    }
  }
  return oss;
}



template StreamCounter& operator<<(StreamCounter& s,  vector<vector<TrackerResult>> const& info);
template StreamCounter& operator<<(StreamCounter& s,  std::vector<cros_reid_info>   const& info);
template StreamCounter& operator<<(StreamCounter& s,  std::vector<cros_reid_info*>  const& info);

