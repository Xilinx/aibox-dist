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

#include <vector>
#include <ostream>
#include <map>
#include "../src/mtmc_reid.hpp"

class StreamCounter
{
    public:
    std::ofstream& GetS() { return _ofs;};
    int GetC() { return _count;};
    void Inc()  { _count++;};
    StreamCounter(std::string name, std::ios_base::openmode mode): _ofs(name, mode), _count(0){};

    private:
    std::ofstream  _ofs;
    int            _count;
};

template<typename T>
class ResultDumper
{
    public:
    static T& Get(std::string name)
    {
        auto it = _ofs.find(name);
        if (it != _ofs.end())
        {
            return it->second;
        }
        else
        {
            auto ret = _ofs.insert(std::pair(name, T(name, std::ofstream::out | std::ofstream::app)));
            if (!ret.second)
            {
                std::string msg(name);
                msg += " as arg to get ofstream cause error.";
                throw std::invalid_argument(msg.c_str());
            }
            return ret.first->second;
        }
    };
    private:
    static std::map<std::string, T> _ofs;

    ResultDumper();
    ~ResultDumper();
};

std::ostream &operator<<(std::ostream& oss, std::vector<vitis::ai::cros_reid_info> const& cams_input_info);
std::ostream &operator<<(std::ostream& oss, std::vector<vitis::ai::cros_reid_info*> const& cams_input_info);
std::ostream &operator<<(std::ostream& oss, vector<vector<vitis::ai::TrackerResult>> const& cams_input_info);

template<typename T>
StreamCounter& operator<<(StreamCounter& s, T const& cams_input_info)
{
  s.GetS() << "Ind: " << s.GetC() << "\n";
  s.Inc();
  s.GetS() << cams_input_info;
  s.GetS() << "------------------------------\n";
  return s;
};
