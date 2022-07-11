/*
 * Copyright 2021 Xilinx Inc.
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

#include <gst/vvas/gstinferencemeta.h>
#include <vvas/vvas_kernel.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vitis/ai/nnpp/reid.hpp>
#include <vitis/ai/reid.hpp>
#include <vitis/ai/classification.hpp>
#include "../cros_mt_reid/src/mtmc_reid.hpp"
#include "common.hpp"

#define MAX_REID 20
#define DEFAULT_REID_THRESHOLD 0.2
#define DEFAULT_MODEL_NAME     "personreid-res18_pt"
#define DEFAULT_MODEL_PATH     "/opt/xilinx/share/vitis_ai_library/models/kv260-aibox-dist"
#define DEFAULT_ORIENTATION_MODEL_NAME "person-orientation_pruned_558m_pt"

enum
{
  LOG_LEVEL_ERROR,
  LOG_LEVEL_WARNING,
  LOG_LEVEL_INFO,
  LOG_LEVEL_DEBUG
};

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG_MESSAGE(level, ...) {\
  do {\
    char *str; \
    if (level == LOG_LEVEL_ERROR)\
      str = (char*)"ERROR";\
    else if (level == LOG_LEVEL_WARNING)\
      str = (char*)"WARNING";\
    else if (level == LOG_LEVEL_INFO)\
      str = (char*)"INFO";\
    else if (level == LOG_LEVEL_DEBUG)\
      str = (char*)"DEBUG";\
    if (!kernel_priv || level <= kernel_priv->debug) {\
      printf("[%s %s:%d] %s: ",__FILENAME__, __func__, __LINE__, str);\
      printf(__VA_ARGS__);\
      printf("\n");\
    }\
  } while (0); \
}


using namespace std;
using namespace vitis::ai;

struct _Face {
  int last_frame_seen;
  int xctr;
  int yctr;
  int id;
  cv::Mat features;
};

typedef struct _kern_priv {
  uint32_t debug;
  double threshold;
  std::string modelpath;
  std::string modelname;
  std::string orienModelname;
  std::shared_ptr<vitis::ai::Reid> det;
  std::shared_ptr<vitis::ai::Classification> orientation;
  uint64_t frame_num;
} ReidKernelPriv;

struct _roi {
    uint32_t y_cord;
    uint32_t x_cord;
    uint32_t height;
    uint32_t width;
    double   prob;
	  GstInferencePrediction *prediction;
};

#define MAX_CHANNELS 40
typedef struct _vvas_ms_roi {
    uint32_t nobj;
    struct _roi roi[MAX_CHANNELS];
} vvas_ms_roi;

static int parse_rect(VVASKernel * handle, int start,
      VVASFrame * input[MAX_NUM_OBJECT], VVASFrame * output[MAX_NUM_OBJECT],
      vvas_ms_roi &roi_data
      )
{
    VVASFrame *inframe = input[0];
    GstInferenceMeta *infer_meta = ((GstInferenceMeta *)gst_buffer_get_meta((GstBuffer *)
                                                              inframe->app_priv,
                                                          gst_inference_meta_api_get_type()));
    roi_data.nobj = 0;
    if (infer_meta == NULL)
    {
        return 0;
    }

    GstInferencePrediction *root = infer_meta->prediction;

    /* Iterate through the immediate child predictions */
    GSList *tmp = gst_inference_prediction_get_children(root);
    for (GSList *child_predictions = tmp;
         child_predictions;
         child_predictions = g_slist_next(child_predictions))
    {
        GstInferencePrediction *child = (GstInferencePrediction *)child_predictions->data;

        /* On each children, iterate through the different associated classes */
        for (GList *classes = child->classifications;
             classes; classes = g_list_next(classes))
        {
            GstInferenceClassification *classification = (GstInferenceClassification *)classes->data;
            if (roi_data.nobj < MAX_CHANNELS)
            {
                int ind = roi_data.nobj;
                struct _roi &roi = roi_data.roi[ind];
                roi.y_cord = (uint32_t)child->bbox.y + child->bbox.y % 2;
                roi.x_cord = (uint32_t)child->bbox.x;
                roi.height = (uint32_t)child->bbox.height - child->bbox.height % 2;
                roi.width = (uint32_t)child->bbox.width - child->bbox.width % 2;
                roi.prob = classification->class_prob;
                roi.prediction = child;
                roi_data.nobj++;

            }
        }
    }
    g_slist_free(tmp);
    return 0;
}

extern "C" {
  static int xsleep = 100;
int32_t xlnx_kernel_init(VVASKernel *handle) {
  char *env = getenv("SLEEP");
  if (env) {
    xsleep = atoi(env);
    printf("SLEEP env: %d\n", xsleep);
  }
  json_t *jconfig = handle->kernel_config;
  json_t *val; /* kernel config from app */

  handle->is_multiprocess = 1;
  ReidKernelPriv *kernel_priv = new ReidKernelPriv;
  if (!kernel_priv) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, "Error: Unable to allocate reID kernel memory\n");
  }

  /* parse config */
  val = json_object_get(jconfig, "threshold");
  if (!val || !json_is_number(val))
    kernel_priv->threshold = DEFAULT_REID_THRESHOLD;
  else
    kernel_priv->threshold = json_number_value(val);

  val = json_object_get(jconfig, "debug_level");
  if (!val || !json_is_number(val))
    kernel_priv->debug = LOG_LEVEL_ERROR;
  else
    kernel_priv->debug = json_number_value(val);

  val = json_object_get(jconfig, "model-name");
  if (!val || !json_is_string (val))
    kernel_priv->modelname = DEFAULT_MODEL_NAME;
  else
    kernel_priv->modelname = (char *) json_string_value (val);

  val = json_object_get(jconfig, "orientation-model-name");
  if (!val || !json_is_string (val))
    kernel_priv->orienModelname = DEFAULT_ORIENTATION_MODEL_NAME;
  else
    kernel_priv->orienModelname = (char *) json_string_value (val);

  val = json_object_get(jconfig, "model-path");
  if (!val || !json_is_string (val))
    kernel_priv->modelpath = DEFAULT_MODEL_PATH;
  else
    kernel_priv->modelpath = (char *) json_string_value (val);

  std::string xmodelfile = kernel_priv->modelpath + "/" + kernel_priv->modelname + "/" + kernel_priv->modelname + ".xmodel";
  kernel_priv->det = vitis::ai::Reid::create(xmodelfile);
  if (kernel_priv->det.get() == NULL) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, "Error: Unable to create Reid runner with model %s.\n", xmodelfile.c_str());
  }

  xmodelfile = kernel_priv->modelpath + "/" + kernel_priv->orienModelname + "/" + kernel_priv->orienModelname + ".xmodel";
  kernel_priv->orientation = vitis::ai::Classification::create(xmodelfile);
  if (kernel_priv->orientation.get() == NULL) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, "Error: Unable to create orientation runner with model %s.\n", xmodelfile.c_str());
  }
  kernel_priv->frame_num = 0;

  handle->kernel_priv = (void *)kernel_priv;
  return 0;
}

uint32_t xlnx_kernel_deinit(VVASKernel *handle) {
  ReidKernelPriv *kernel_priv = (ReidKernelPriv *)handle->kernel_priv;
  if (kernel_priv) {
    delete (kernel_priv);
  }
  return 0;
}

int32_t xlnx_kernel_start(VVASKernel *handle, int start /*unused */,
                          VVASFrame *input[MAX_NUM_OBJECT],
                          VVASFrame *output[MAX_NUM_OBJECT]) {
  VVASFrame *in_vvas_frame = input[0];
  ReidKernelPriv *kernel_priv = (ReidKernelPriv *)handle->kernel_priv;
  if ( !kernel_priv->det.get() || !kernel_priv->orientation.get() ) {
    return 1;
  }

  cros_reid_info * cros_info = new cros_reid_info;
  cros_info->frame_id = kernel_priv->frame_num;
  kernel_priv->frame_num++;

  vvas_ms_roi roi_data;
  parse_rect(handle, start, input, output, roi_data);

  m__TIC__(getfeat);
  LOG_MESSAGE(LOG_LEVEL_DEBUG, "roi size: %d", roi_data.nobj);
  for (uint32_t i = 0; i < roi_data.nobj; i++) {
    struct _roi& roi = roi_data.roi[i];
    {
      GstBuffer *buffer = (GstBuffer *)roi.prediction->sub_buffer; /* resized crop image*/
      int xctr = roi.x_cord + roi.width / 2;
      int yctr = roi.y_cord + roi.height / 2;
      GstMapInfo info;
      gst_buffer_map(buffer, &info, GST_MAP_READ);

      GstVideoMeta *vmeta = gst_buffer_get_video_meta(buffer);
      if (!vmeta) {
        LOG_MESSAGE(LOG_LEVEL_ERROR, "ERROR: VVAS REID: video meta not present in buffer");
      } else if (vmeta->width == 80 && vmeta->height == 176) {
        char *indata = (char *)info.data;
        cv::Mat image(vmeta->height, vmeta->width, CV_8UC3, indata);
        auto input_box =
            cv::Rect2f(roi.x_cord, roi.y_cord,
                       roi.width, roi.height);
        m__TIC__(reidrun);
        auto reid_result = kernel_priv->det->run(image);
        auto feat = reid_result.feat;
        m__TOC__(reidrun);

        auto ori = kernel_priv->orientation->run(image).scores[0].index;

        m__TIC__(inputpush);
        person_info pinfo = {feat, {roi.x_cord, roi.y_cord,
                       roi.width, roi.height}, roi.prob, -1, ori, 0};
        LOG_MESSAGE(LOG_LEVEL_DEBUG, "orientation: %d\n", ori);
        cros_info->person_infos.emplace_back(pinfo);
        m__TOC__(inputpush);
        LOG_MESSAGE(LOG_LEVEL_DEBUG, "Tracker input: Frame %lu: obj_ind %d, xmin %u, ymin %u, xmax %u, ymax %u, prob: %f \n \
            imgw=%d imgh=%d\n",
                    kernel_priv->frame_num, i, roi.x_cord, roi.y_cord,
                       roi.x_cord + roi.width,
                       roi.y_cord + roi.height, roi.prob
                       , image.cols, image.rows
                       );
      } else {
        LOG_MESSAGE(LOG_LEVEL_ERROR, "ERROR: VVAS REID: Invalid resolution for reid (%u x %u)\n",
               vmeta->width, vmeta->height);
      }
      gst_buffer_unmap(buffer, &info);
    }
  }
  usleep(xsleep);
  m__TOC__(getfeat);
  {
    VVASFrame *inframe = input[0];
    GstInferenceMeta *infer_meta = ((GstInferenceMeta *)gst_buffer_get_meta((GstBuffer *)
                                                              inframe->app_priv,
                                                          gst_inference_meta_api_get_type()));
    LOG_MESSAGE(LOG_LEVEL_DEBUG, "infer ptr: %p\n", cros_info);
    if (!infer_meta) {
      GstBuffer *writable_buf = gst_buffer_make_writable ((GstBuffer *)inframe->app_priv);
      infer_meta = (GstInferenceMeta *) gst_buffer_add_meta (writable_buf, gst_inference_meta_get_info (), NULL);
    }

    if (infer_meta && infer_meta->prediction) {
      infer_meta->prediction->reserved_3 = (void*)cros_info;
      LOG_MESSAGE(LOG_LEVEL_DEBUG, "data fr id: %lu\n", (uint64)infer_meta->prediction->reserved_3);
      LOG_MESSAGE(LOG_LEVEL_DEBUG, "reid reserve ptr: %p\n", infer_meta->prediction->reserved_3);
      LOG_MESSAGE(LOG_LEVEL_DEBUG, "reid person size: %ld\n", ((struct cros_reid_info *)infer_meta->prediction->reserved_3)->person_infos.size() );
    } else {
      assert(0);
    }
  }

  return 0;
}

int32_t xlnx_kernel_done(VVASKernel *handle) {
  /* dummy */
  return 0;
}
}
