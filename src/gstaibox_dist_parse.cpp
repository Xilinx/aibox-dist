/*
 * Copyright (C) 2020 - 2021 Xilinx, Inc.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
 * KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL XILINX BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. Except as contained in this notice, the name of the Xilinx shall
 * not be used in advertising or otherwise to promote the sale, use or other
 * dealings in this Software without prior written authorization from Xilinx.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/video.h>
#include <string.h>
#include <gst/vvas/gstinferencemeta.h>
#include <stdlib.h>
#include <stdio.h>
#include <cassert>
#include <vector>
#include <inttypes.h>
#include "dataSaveLoad.hpp"
#include "gstaibox_dist_parse.hpp"

using namespace vitis::ai;
#define RESULT_ALL_SEI 0

GST_DEBUG_CATEGORY_STATIC (gst_aibox_dist_parse_debug_category);
#define GST_CAT_DEFAULT gst_aibox_dist_parse_debug_category
#define GSTAIBOX_DIST_PARSE_DEFAULT_MAX_NUM 15
#define GSTAIBOX_DIST_PARSE_DEFAULT_USE_PTS 0
#define GSTAIBOX_DIST_PARSE_DEFAULT_HOST ""
#define gst_aibox_dist_parse_parent_class parent_class

static GstFlowReturn gst_aibox_dist_parse_transform_ip (GstBaseTransform * trans,
    GstBuffer * buf);
static void gst_aibox_dist_parse_finalize (GObject * gobject);
static gboolean gst_aibox_dist_parse_stop(GstBaseTransform * trans);
static gboolean gst_aibox_dist_parse_start(GstBaseTransform * trans);

G_DEFINE_TYPE_WITH_CODE (GstAIBOX_DIST_Parse, gst_aibox_dist_parse,
    GST_TYPE_BASE_TRANSFORM,
    GST_DEBUG_CATEGORY_INIT (gst_aibox_dist_parse_debug_category, "aibox_dist_parse", 0,
        "debug category for aibox parser element"));

enum {
  PROP_0,
  PROP_HOST,
  PROP_ROI_MAX_NUM,
  PROP_USE_PTS,
  PROP_RESULT_LIMIT
};

/* <timestamp in uint64> + <num rois in uint> */
#define VVAS_ROI_SEI_EXTRA_INFO_SIZE (sizeof(guint64)+sizeof (guint))


static void
gst_aibox_dist_parse_set_property (GObject * object,
    guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (object);

  switch (prop_id) {
    case PROP_ROI_MAX_NUM:
      plug->max_num = g_value_get_uint64 (value);
      break;
    case PROP_USE_PTS:
      plug->use_pts = g_value_get_uint64 (value);
      break;
    case PROP_HOST:
      plug->host = g_strdup(g_value_get_string(value));
      break;
    case PROP_RESULT_LIMIT:
      plug->result_limit = g_value_get_uint64 (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_aibox_dist_parse_get_property (GObject * object,
    guint prop_id, GValue * value, GParamSpec * pspec)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (object);

  switch (prop_id) {
    case PROP_ROI_MAX_NUM:
      g_value_set_uint64 (value, plug->max_num);
      break;
    case PROP_HOST:
      g_value_set_string(value, plug->host);
      break;
    case PROP_RESULT_LIMIT:
      g_value_set_uint64 (value, plug->result_limit);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}


static gboolean
handle_sei_info (GstElement* self, GstEvent * event)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (self);
  const GstStructure *s;
  guint32 payload_type;
  GstBuffer *buf;
  GstMapInfo map;
  s = gst_event_get_structure (event);

  if (!gst_structure_get (s, "payload-type", G_TYPE_UINT, &payload_type,
          "payload", GST_TYPE_BUFFER, &buf, NULL)) {
    GST_ERROR_OBJECT (plug, "CH %lu: ERROR: Failed to parse event\n", plug->max_num);
    return TRUE;
  }
  if (payload_type == 78) {
    GST_DEBUG_OBJECT(plug, "CH %lu: Payload type : 78\n", plug->max_num);
    if (!gst_buffer_map (buf, &map, GST_MAP_READ)) {
      GST_WARNING_OBJECT (self, "ERROR: Failed to map payload buffer");
      gst_buffer_unref (buf);
      return TRUE;
    }

    GST_DEBUG_OBJECT (self, "CH %lu: Requesting  (payload-type=%d)", plug->max_num, payload_type);

    if (RESULT_ALL_SEI)
    {
      struct cros_reid_info* tmpinfo = new struct cros_reid_info;
      std::istringstream iss(std::string((char*)map.data, map.size));
      deserializeData(*tmpinfo, iss) ;
      for (int i = 0; i < tmpinfo->person_infos.size(); i++)
      {
          struct person_info& tmp = tmpinfo->person_infos[i];
      }
      {
        const std::lock_guard<std::mutex> lock(plug->data->mtx);
        plug->data->data_from_event.push(tmpinfo);
      }
    }
    else
    {
      std::istringstream iss(std::string((char*)map.data, map.size));
      uint64_t frameid = *(uint64_t*)map.data;
      uint64_t timestamp = *(uint64_t*)(map.data+sizeof(uint64_t));
      {
        const std::lock_guard<std::mutex> lock(plug->data->mtx);
        plug->data->frame_ind_queue.push( {.frame_ind = frameid, .timestamp = timestamp} );
      }
      GST_DEBUG_OBJECT (plug, "CH %lu, SEI FR %lu, TS %lu.", plug->max_num, frameid, timestamp);
    }
    gst_buffer_unmap (buf, &map);
  } else {
    GST_WARNING_OBJECT (self,
        "Payload type is not matching to draw boundign box.");
  }

  gst_buffer_unref (buf);

  return TRUE;
}

#define OMX_ALG_GST_EVENT_INSERT_PREFIX_SEI "omx-alg/sei-parsed"
static gboolean
gst_aibox_dist_parse_sink_event(GstBaseTransform * btrans, GstEvent * event)
{
  GstAIBOX_DIST_Parse *self;
  self = GST_AIBOX_DIST_PARSE(btrans);
  if (gst_event_has_name (event, OMX_ALG_GST_EVENT_INSERT_PREFIX_SEI)) {
    GST_DEBUG_OBJECT (self, "aibox_dist_parse: omx event received\n");
    handle_sei_info (GST_ELEMENT_CAST(self), event);
  } else {
    switch (GST_EVENT_TYPE (event)) {
     case GST_EVENT_SEGMENT:{
      GST_OBJECT_LOCK (self);
      gst_event_copy_segment (event, &self->segment);
      GST_DEBUG_OBJECT (self, "configured segment %" GST_SEGMENT_FORMAT,
          &self->segment);
      GST_OBJECT_UNLOCK (self);
     }
     default:
     break;
    }
  }
  return GST_BASE_TRANSFORM_CLASS(parent_class)->sink_event (btrans, event);
}

static void
gst_aibox_dist_parse_class_init (GstAIBOX_DIST_ParseClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *transform_class = GST_BASE_TRANSFORM_CLASS (klass);

  gobject_class->set_property = gst_aibox_dist_parse_set_property;
  gobject_class->get_property = gst_aibox_dist_parse_get_property;
  gobject_class->finalize = gst_aibox_dist_parse_finalize;

  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_pad_template_new ("src", GST_PAD_SRC, GST_PAD_ALWAYS,
          gst_caps_from_string (GST_VIDEO_CAPS_MAKE (GST_VIDEO_FORMATS_ALL))));
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_pad_template_new ("sink", GST_PAD_SINK, GST_PAD_ALWAYS,
          gst_caps_from_string (GST_VIDEO_CAPS_MAKE (GST_VIDEO_FORMATS_ALL))));

  g_object_class_install_property (G_OBJECT_CLASS (klass), PROP_ROI_MAX_NUM,
      g_param_spec_uint64 ("max-num", "Maximum number of ROIs",
          "Max Number of ROIs to be attached to metadata",
          0, G_MAXUINT, GSTAIBOX_DIST_PARSE_DEFAULT_MAX_NUM,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (G_OBJECT_CLASS (klass), PROP_USE_PTS,
      g_param_spec_uint64 ("use-pts", "use pts as timestamp for sync",
          "Use pts as timestamp for sync",
          0, G_MAXUINT, GSTAIBOX_DIST_PARSE_DEFAULT_USE_PTS,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));


  g_object_class_install_property (G_OBJECT_CLASS (klass), PROP_RESULT_LIMIT,
      g_param_spec_uint64 ("result-limit", "max num of result buffer",
          "max num of result buffer",
          0, G_MAXUINT, 10000,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (G_OBJECT_CLASS (klass), PROP_HOST,
      g_param_spec_string("host", "Host",
          "Hostname or ip of the camera",
          GSTAIBOX_DIST_PARSE_DEFAULT_HOST,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "AIBox meta data",
      "Video/Filter", "AIBox Metadata ",
      "Xilinx Inc");

  transform_class->transform_ip =
      GST_DEBUG_FUNCPTR (gst_aibox_dist_parse_transform_ip);
  transform_class->start = gst_aibox_dist_parse_start;
  transform_class->stop = gst_aibox_dist_parse_stop;

  transform_class->sink_event = GST_DEBUG_FUNCPTR (gst_aibox_dist_parse_sink_event);;
}

static void
zthread_rec_msg(GstAIBOX_DIST_Parse * plug)
{
  while(!plug->data->zthreadStop) {
    zmq::message_t update;
    if ( plug->data->zsck.recv(&update) == 0 ) {
      continue;
    }

    std::string msg(static_cast<char*>(update.data()), update.size());

    std::istringstream iss(msg);
    struct cros_reid_info* tmpinfo = new struct cros_reid_info;
    deserializeData(*tmpinfo, iss) ;

    GST_DEBUG_OBJECT (plug, "New data: CH%lu %" PRIu64 ", %p\n", plug->max_num, tmpinfo->frame_id, tmpinfo);
    {
      std::lock_guard<std::mutex> lk(plug->data->mtx);
      plug->data->results.insert(std::pair<int, vitis::ai::cros_reid_info*>(tmpinfo->frame_id,tmpinfo));
      GST_DEBUG_OBJECT(plug, "CH%lu Remain Data size after recv: %lu\n", plug->max_num, plug->data->results.size());
      if (plug->data->results.size() > plug->result_limit) {
        GST_DEBUG_OBJECT (plug, "CH%lu remove result %lu, %p.", plug->max_num,
          plug->data->results.begin()->first, plug->data->results.begin()->second);
        delete plug->data->results[plug->data->results.begin()->first];
        plug->data->results.erase(plug->data->results.begin());
        GST_DEBUG_OBJECT(plug, "CH%lu Erase over large data set to: %lu\n", plug->max_num, plug->data->results.size());
      }
    }
    GST_DEBUG_OBJECT(plug, "goto next recv \n");
  }
  GST_DEBUG_OBJECT(plug, "rec thread quiting\n");
}

static void
gst_aibox_dist_parse_init (GstAIBOX_DIST_Parse * plug)
{
  plug->data = new ParseData;
  plug->pre_ind = 0;
  plug->pre_timestamp = 0;
  plug->first_timestamp = 0;
  plug->num_frame = 0;
  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (plug), TRUE);
}

_ParseData::~_ParseData()
{
  if (RESULT_ALL_SEI)
  {
    assert(results.size() == 0);
    while(!data_from_event.empty())
    {
      auto p = data_from_event.front();
      delete p;
      data_from_event.pop();
    }
  }
  else
  {
    assert(data_from_event.size() == 0);
    for (auto r : results)
    {
      auto p = r.second;
      delete p;
    }
  }
}

static void
gst_aibox_dist_parse_finalize (GObject * gobject)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (gobject);
  plug->data->max_num = plug->max_num;
  delete plug->data;
  plug->data = NULL;
  free(plug->host);
  plug->host = NULL;
  G_OBJECT_CLASS (parent_class)->finalize (gobject);
}

#define DO_RUNNING_AVG(avg,val,size) (((val) + ((size)-1) * (avg)) / (size))

#define UPDATE_RUNNING_AVG(avg,val)   DO_RUNNING_AVG(avg,val,8)
#define UPDATE_RUNNING_AVG_P(avg,val)   DO_RUNNING_AVG(avg,val,16)
#define UPDATE_RUNNING_AVG_N(avg,val)   DO_RUNNING_AVG(avg,val,4)

static gboolean
gst_aibox_dist_parse_start(GstBaseTransform *trans)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (trans);
  plug->data->zctx = zmq::context_t(1); 
  plug->data->zsck = zmq::socket_t(plug->data->zctx, ZMQ_SUB);
  std::ostringstream oss;
  oss << "tcp://" << plug->host << ":" << 5550;
  plug->data->zsck.connect(oss.str().c_str());
  plug->data->zsck.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);
  int timeout = 100;
  plug->data->zsck.setsockopt(ZMQ_RCVTIMEO, timeout);
  plug->data->zthreadStop = false;
  plug->data->zthread = std::thread(zthread_rec_msg, plug);
  return TRUE;
}

static gboolean
gst_aibox_dist_parse_stop(GstBaseTransform * trans)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (trans);
  plug->data->zthreadStop = true;
  plug->data->zthread.join();
  plug->data->zsck.close();
  plug->data->zctx.close();
  return TRUE;
}

static GstFlowReturn
gst_aibox_dist_parse_transform_ip (GstBaseTransform * trans, GstBuffer * buf)
{
  GstAIBOX_DIST_Parse *plug = GST_AIBOX_DIST_PARSE (trans);
  GstInferenceMeta *infer_meta = NULL;
  GstInferencePrediction *root, *child;
  GstInferenceClassification *classification;
  GSList *child_predictions;
  GList *classes;
  guint roi_count = 0;

  infer_meta = ((GstInferenceMeta *) gst_buffer_get_meta ((GstBuffer *)
          buf, gst_inference_meta_api_get_type ()));

  if (!infer_meta) {
    infer_meta = (GstInferenceMeta *) gst_buffer_add_meta ((GstBuffer *)buf,
      gst_inference_meta_get_info (), NULL);
    if(infer_meta->prediction) {
    infer_meta->prediction->reserved_1 = NULL;
    infer_meta->prediction->reserved_2 = NULL;
    infer_meta->prediction->reserved_3 = NULL;
    infer_meta->prediction->reserved_4 = NULL;
    infer_meta->prediction->reserved_5 = NULL;
    } else {
      GST_INFO_OBJECT (plug, "New infer meta has no prediction.");
    }
  } else {
    GST_INFO_OBJECT (plug, "Parsed buffer already has infer_meta.");
  }

  if (RESULT_ALL_SEI)
  {
    const std::lock_guard<std::mutex> lock(plug->data->mtx);
    if (plug->data->data_from_event.size() > 0) {
      if (!infer_meta) {
        GST_ERROR_OBJECT (plug, "failed to add meta to buffer.");
          delete plug->data->data_from_event.front();
          plug->data->data_from_event.pop();
      } else {
        if(infer_meta->prediction && infer_meta->prediction->reserved_3 == NULL) {
          infer_meta->prediction->reserved_3 = (void*)(plug->data->data_from_event.front());
          plug->data->data_from_event.pop();
        } else {
          delete plug->data->data_from_event.front();
          plug->data->data_from_event.pop();
          assert(0);
        }
      }
    } else {
      GST_ERROR_OBJECT (plug, "No parsed data for this buffer available now.");
    }
  }
  else
  {
    uint64_t frameid = 0, timestamp = 0;
    {
      const std::lock_guard<std::mutex> lock(plug->data->mtx);
      if ( plug->data->frame_ind_queue.size() ) {
        auto data = plug->data->frame_ind_queue.front();
        frameid = data.frame_ind;
        timestamp = data.timestamp;
        plug->pre_ind = frameid;
        if (plug->num_frame == 0) {
          plug->first_timestamp = timestamp;
        }
        plug->pre_timestamp = timestamp;

        plug->data->frame_ind_queue.pop();
      } else {
        frameid = ++plug->pre_ind;
        GST_ERROR_OBJECT (plug, "failed to get Frame ID. CH %lu. Assume %lu.", plug->max_num, frameid);
      }
    
      if (plug->use_pts) {
        timestamp = GST_BUFFER_PTS (buf);
      }
    }

    GST_DEBUG_OBJECT (plug, "Finding results for Frame ID %lu. CH %lu.", frameid, plug->max_num);
    vitis::ai::cros_reid_info* curResult = NULL;
    {
      const std::lock_guard<std::mutex> lock(plug->data->mtx);
      std::map<uint64_t, vitis::ai::cros_reid_info*>::iterator it = plug->data->results.find(frameid);
      if (it != plug->data->results.end()) {
        if (!infer_meta) {
          GST_ERROR_OBJECT (plug, "failed to add meta to buffer.");
        } else {
          curResult = it->second;
          for(auto iold = plug->data->results.begin(); iold != it; iold++)
          {
            delete iold->second;
          }
          plug->data->results.erase(plug->data->results.begin(), it);
          plug->data->results.erase(it);
        }
        GST_DEBUG_OBJECT (plug, "Parsed data for CH %lu, FR %lu buffer available now: %p.", plug->max_num, frameid, curResult);
      } else {
        GST_DEBUG_OBJECT (plug, "No parsed data for CH %lu, FR %lu buffer available now.", plug->max_num, frameid);
      }
    }

    if(infer_meta->prediction && infer_meta->prediction->reserved_3 == NULL) {
      infer_meta->prediction->reserved_3 = (void*)(curResult);
      infer_meta->prediction->reserved_4 = (void*)(new uint64_t(timestamp) );
      infer_meta->prediction->reserved_2 = (void*)(plug->max_num);
    } else {
      GST_ERROR_OBJECT (plug, "Bf meta invalid: CH %lu, FR %lu. Infer_meta->pred: %p, reserv3: %p",
       plug->max_num, frameid, infer_meta->prediction, infer_meta->prediction? infer_meta->prediction->reserved_3: NULL);
      assert(0);
    }
  }
  /* send custom event, so that encoder will insert SEI packets in byte stream */
  return GST_FLOW_OK;
}

static gboolean
aibox_dist_parse_init (GstPlugin * aibox_dist_parse)
{
  return gst_element_register (aibox_dist_parse, "aibox_dist_parse",
      GST_RANK_NONE, GST_TYPE_AIBOX_DIST_PARSE);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "aibox_dist_parse"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    aibox_dist_parse,
    "Xilinx AIBox-dist result parser plugin",
    aibox_dist_parse_init, "1.0", "MIT/X11",
    "Xilinx AIBox-dist result parser plugin", "http://xilinx.com/")
