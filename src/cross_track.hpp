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
#include "../cros_mt_reid/src/mtmc_reid.hpp"
using namespace vitis::ai;
using namespace std;

class CrossTracker
{
    public:
        CrossTracker(std::string confFile, int num, int w, int h);
        ~CrossTracker() {};
        vector<vector<TrackerResult>> Run(vector<cros_reid_info*>& input_infos);
        vector<vector<TrackerResult>> Run(vector<cros_reid_info>& input_infos) ;
        bool ConstrIsOK() { return constrOK; }

    private:
        CrossTracker()=delete;

        vector<MotTracker> motTrackers;
        std::shared_ptr<CrossCameraTracker> trackSys;
        vector<int> cams;
        vector<camInfo> camInfos;
        bool constrOK;
};
