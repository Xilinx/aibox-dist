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

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/impl/basic_text_oarchive.ipp>
#include <boost/archive/impl/text_oarchive_impl.ipp>
#include <boost/serialization/vector.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <iostream>
#include "dataSaveLoad.hpp"

using namespace cv;
using namespace std;
using namespace vitis::ai;
using namespace boost::serialization;


namespace boost {
namespace serialization {
template<class Archive>
void serialize(Archive &ar, cv::Mat& mat, const unsigned int)
{
    int cols, rows, type;
    bool continuous;

    if (Archive::is_saving::value) {
        cols = mat.cols; rows = mat.rows; type = mat.type();
        continuous = mat.isContinuous();
    }

    ar & cols & rows & type & continuous;

    if (Archive::is_loading::value)
        mat.create(rows, cols, type);

    if (continuous) {
        const unsigned int data_size = rows * cols * mat.elemSize();
        ar & boost::serialization::make_array(mat.ptr(), data_size);
    } else {
        const unsigned int row_size = cols*mat.elemSize();
        for (int i = 0; i < rows; i++) {
            ar & boost::serialization::make_array(mat.ptr(i), row_size);
        }
    }
}

template<class Archive>
void serialize(Archive &ar, struct person_info & data, const unsigned int)
{
    serialize(ar, data.reid_feat, 0);
    ar & data.bbox & data.score & data.label & data.oritenidx & data.local_id;
}

template<class Archive>
void serialize(Archive &ar, struct cros_reid_info& data, const unsigned int)
{
    ar & data.frame_id & data.person_infos;
}
}
}

void serializeData(ostringstream& oss, struct cros_reid_info& data) {
    boost::archive::binary_oarchive oa(oss);
    oa & data;
}

void deserializeData(struct cros_reid_info& data, istringstream& iss) {
    boost::archive::binary_iarchive ia(iss);
    ia & data;
}
