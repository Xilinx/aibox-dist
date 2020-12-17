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
#include <vector>
#include <zmq.hpp>
#include "gstaibox_dist_attach.hpp"
#include "dataSaveLoad.hpp"

using namespace vitis::ai;
#define RESULT_ALL_SEI 0

GST_DEBUG_CATEGORY_STATIC (gst_aibox_dist_attach_debug_category);
#define GST_CAT_DEFAULT gst_aibox_dist_attach_debug_category
#define GSTAIBOX_DIST_ATTACH_DEFAULT_MAX_NUM 15
#define gst_aibox_dist_attach_parent_class parent_class

static GstFlowReturn gst_aibox_dist_attach_transform_ip (GstBaseTransform * trans,
    GstBuffer * buf);
static void gst_aibox_dist_attach_finalize (GObject * gobject);

G_DEFINE_TYPE_WITH_CODE (GstAIBOX_DIST_Attach, gst_aibox_dist_attach,
    GST_TYPE_BASE_TRANSFORM,
    GST_DEBUG_CATEGORY_INIT (gst_aibox_dist_attach_debug_category, "aibox_dist_attach", 0,
        "debug category for aibox_dist attach element"));

enum {
  PROP_0,
  PROP_ROI_MAX_NUM
};

/* <timestamp in uint64> + <num rois in uint> */
#define VVAS_ROI_SEI_EXTRA_INFO_SIZE (sizeof(guint64)+sizeof (guint))

static void
gst_aibox_dist_attach_set_property (GObject * object,
    guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GstAIBOX_DIST_Attach *plug = GST_AIBOX_DIST_ATTACH (object);

  switch (prop_id) {
    case PROP_ROI_MAX_NUM:
      plug->max_num = g_value_get_uint (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_aibox_dist_attach_get_property (GObject * object,
    guint prop_id, GValue * value, GParamSpec * pspec)
{
  GstAIBOX_DIST_Attach *plug = GST_AIBOX_DIST_ATTACH (object);

  switch (prop_id) {
    case PROP_ROI_MAX_NUM:
      g_value_set_uint (value, plug->max_num);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_aibox_dist_attach_class_init (GstAIBOX_DIST_AttachClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *transform_class = GST_BASE_TRANSFORM_CLASS (klass);

  gobject_class->set_property = gst_aibox_dist_attach_set_property;
  gobject_class->get_property = gst_aibox_dist_attach_get_property;
  gobject_class->finalize = gst_aibox_dist_attach_finalize;

  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_pad_template_new ("src", GST_PAD_SRC, GST_PAD_ALWAYS,
          gst_caps_from_string (GST_VIDEO_CAPS_MAKE (GST_VIDEO_FORMATS_ALL))));
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS (klass),
      gst_pad_template_new ("sink", GST_PAD_SINK, GST_PAD_ALWAYS,
          gst_caps_from_string (GST_VIDEO_CAPS_MAKE (GST_VIDEO_FORMATS_ALL))));

  g_object_class_install_property (G_OBJECT_CLASS (klass), PROP_ROI_MAX_NUM,
      g_param_spec_uint ("roi-max-num", "Maximum number of ROIs",
          "Max Number of ROIs to be attached to metadata",
          0, G_MAXUINT, GSTAIBOX_DIST_ATTACH_DEFAULT_MAX_NUM,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS (klass),
      "AIBox meta data",
      "Video/Filter", "AIBox Metadata ",
      "Xilinx Inc");

  transform_class->transform_ip =
      GST_DEBUG_FUNCPTR (gst_aibox_dist_attach_transform_ip);
}

static void
gst_aibox_dist_attach_init (GstAIBOX_DIST_Attach * plug)
{
  gst_base_transform_set_in_place (GST_BASE_TRANSFORM (plug), TRUE);
  plug->zctx = zmq::context_t(1); 
  plug->zsck = zmq::socket_t(plug->zctx, ZMQ_PUB);
  std::ostringstream oss;
  oss << "tcp://*:" << 5550;
  plug->zsck.bind(oss.str().c_str());
}

static void
gst_aibox_dist_attach_finalize (GObject * gobject)
{
  GstAIBOX_DIST_Attach *plug = GST_AIBOX_DIST_ATTACH (gobject);
  plug->zsck.close();
  G_OBJECT_CLASS (parent_class)->finalize (gobject);
}

static GstFlowReturn
gst_aibox_dist_attach_transform_ip (GstBaseTransform * trans, GstBuffer * buf)
{
  GstAIBOX_DIST_Attach *plug = GST_AIBOX_DIST_ATTACH (trans);
  GstInferenceMeta *infer_meta = NULL;

  infer_meta = ((GstInferenceMeta *) gst_buffer_get_meta ((GstBuffer *)
          buf, gst_inference_meta_api_get_type ()));

  if (!infer_meta) {
    GstBuffer *writable_buf = gst_buffer_make_writable (buf);
    infer_meta = (GstInferenceMeta *) gst_buffer_add_meta (writable_buf,
      gst_inference_meta_get_info (), NULL);
    if (infer_meta) {
      GST_ERROR_OBJECT (plug, "Meta add successfully \n");
    } else {
      GST_ERROR_OBJECT (plug, "Fail to add Meta \n");
    }
  }
  GST_LOG_OBJECT (plug, "infer_meta %p", infer_meta);

  struct cros_reid_info* data = (struct cros_reid_info*)infer_meta->prediction->reserved_3;
  /* send custom event, so that encoder will insert SEI packets in byte stream */
  char* env = getenv("ATTACH_COUNT");
  int count = env ? atoi(env) : 0;
  if (!data) {
    g_message("Attach set for no cros data\n");
    static int idx = 0;
    struct cros_reid_info *data = new struct cros_reid_info;
    data->frame_id = idx++;
    data->person_infos.resize(count);
    for (int i = 0; i < count; i++) {
      data->person_infos[i].bbox.resize(4);
      data->person_infos[i].bbox[0] = i;
      data->person_infos[i].bbox[1] = i+1;
      data->person_infos[i].bbox[2] = i+2;
      data->person_infos[i].bbox[3] = i+3;
    }

    infer_meta->prediction->reserved_3 = (void*)data;
  }

  data = (struct cros_reid_info*)infer_meta->prediction->reserved_3;
  if (data) {
    GstBuffer *sei_buf = NULL;
    gchar *sei_data = NULL;
    guint offset = 0;
    uint64_t count = data->person_infos.size();
    GstStructure *s = NULL;
    GstEvent *event = NULL;

    std::ostringstream oss;
    serializeData(oss, *data);

    if(!RESULT_ALL_SEI)
    {
      gsize msgSize = oss.str().size();
      zmq::message_t message(msgSize);
      memcpy ((void *) message.data (), oss.str().c_str(), msgSize);
      GST_DEBUG_OBJECT (plug, "zmp send msg: \n");
      plug->zsck.send(message);


      {
        gsize sei_size = sizeof(uint64_t) * 2;
        sei_data = (gchar *) g_malloc0 (sei_size);
        if (!sei_data) {
          GST_ERROR_OBJECT (plug, "FR: %ld, failed to allocate SEI memory", data->frame_id);
          return GST_FLOW_ERROR;
        }

        memcpy (sei_data, &(data->frame_id), sizeof(data->frame_id));

        struct timespec _t;
        clock_gettime(CLOCK_REALTIME, &_t);
        uint64_t tmptime = _t.tv_sec*1000 + lround(_t.tv_nsec/1e6);
        GST_DEBUG_OBJECT (plug, "gst clock: %ld, %ld, %ld \n", _t.tv_sec, _t.tv_nsec, tmptime);

        memcpy (sei_data + sizeof(uint64), &(tmptime), sizeof(tmptime));

        sei_buf = gst_buffer_new_wrapped_full(GstMemoryFlags(0), sei_data, sei_size, 0, sei_size,
                                              sei_data, g_free);
        GST_DEBUG_OBJECT(plug, "FR: %ld, attach person count: %ld, data size: %ld ",
                         data->frame_id, count, sei_size);

        s = gst_structure_new("omx-alg/insert-suffix-sei",
                              "payload-type", G_TYPE_UINT, 78,
                              "payload", GST_TYPE_BUFFER, sei_buf, NULL);

        event = gst_event_new_custom(GST_EVENT_CUSTOM_DOWNSTREAM, s);
        if (!gst_pad_push_event(GST_BASE_TRANSFORM_SRC_PAD(trans), event))
        {
          GST_ERROR_OBJECT(plug, "failed to send custom SEI event");
          gst_buffer_unref(sei_buf);
          return GST_FLOW_ERROR;
        }

        GST_DEBUG_OBJECT(plug, "sent SEI event with SEI payload size %lu",
                         sei_size);
        gst_buffer_unref(sei_buf);
      }
    }
    else
    {
      gsize sei_size = oss.str().size();
      sei_data = (gchar *) g_malloc0 (sei_size);
      if (!sei_data) {
        GST_ERROR_OBJECT (plug, "FR: %ld, failed to allocate SEI memory", data->frame_id);
        return GST_FLOW_ERROR;
      }
      memcpy (sei_data, oss.str().c_str(), sei_size);

      sei_buf = gst_buffer_new_wrapped_full (GstMemoryFlags(0), sei_data, sei_size, 0, sei_size,
          sei_data, g_free);
      GST_DEBUG_OBJECT (plug, "FR: %ld, attach person count: %ld, data size: %ld ",
          data->frame_id, count, sei_size);

      s = gst_structure_new ("omx-alg/insert-suffix-sei",
          "payload-type", G_TYPE_UINT, 78,
          "payload", GST_TYPE_BUFFER, sei_buf, NULL);

      event = gst_event_new_custom (GST_EVENT_CUSTOM_DOWNSTREAM, s);

      if (!gst_pad_push_event (GST_BASE_TRANSFORM_SRC_PAD (trans), event)) {
        GST_ERROR_OBJECT (plug, "failed to send custom SEI event");
        gst_buffer_unref (sei_buf);
        return GST_FLOW_ERROR;
      }

      GST_DEBUG_OBJECT (plug, "sent SEI event with SEI payload size %lu",
          sei_size);
      gst_buffer_unref (sei_buf);
    }
    delete data;
    infer_meta->prediction->reserved_3 = NULL;
  }

  return GST_FLOW_OK;
}

static gboolean
aibox_dist_attach_init (GstPlugin * aibox_dist_attach)
{
  return gst_element_register (aibox_dist_attach, "aibox_dist_attach",
      GST_RANK_NONE, GST_TYPE_AIBOX_DIST_ATTACH);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "aibox_dist_attach"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    aibox_dist_attach,
    "Xilinx AIBox result attacher plugin",
    aibox_dist_attach_init, "1.0", "MIT/X11",
    "Xilinx AIBox result attacher plugin", "http://xilinx.com/")
