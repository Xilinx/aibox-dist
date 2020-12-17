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

#ifndef _GST_AIBOX_DIST_PARSE_H_
#define _GST_AIBOX_DIST_PARSE_H_

#include <gst/base/gstbasetransform.h>
#include <queue>
#include <thread>
#include <mutex>
#include <zmq.hpp>
#include <map>
#include "../cros_mt_reid/src/mtmc_reid.hpp"

G_BEGIN_DECLS

#define GST_TYPE_AIBOX_DIST_PARSE   (gst_aibox_dist_parse_get_type())
#define GST_AIBOX_DIST_PARSE(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_AIBOX_DIST_PARSE,GstAIBOX_DIST_Parse))
#define GST_AIBOX_DIST_PARSE_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_AIBOX_DIST_PARSE,GstAIBOX_DIST_ParseClass))
#define GST_IS_AIBOX_DIST_PARSE(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_AIBOX_DIST_PARSE))
#define GST_IS_AIBOX_DIST_PARSE_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_AIBOX_DIST_PARSE))

typedef struct _GstAIBOX_DIST_Parse GstAIBOX_DIST_Parse;
typedef struct _GstAIBOX_DIST_ParseClass GstAIBOX_DIST_ParseClass;

typedef struct _Frame_ind_info
{
  uint64_t frame_ind;
  uint64_t timestamp;
} Frame_ind_info;

typedef struct _ParseData
{
  ~_ParseData();
  std::queue<vitis::ai::cros_reid_info *> data_from_event;
  std::queue<Frame_ind_info> frame_ind_queue;
  std::mutex mtx;
  zmq::context_t zctx;
  zmq::socket_t zsck;
  gboolean  zthreadStop;
  std::thread zthread;
  std::map<uint64_t, vitis::ai::cros_reid_info*> results;
  guint64 max_num; //debug
} ParseData;

struct _GstAIBOX_DIST_Parse
{
  GstBaseTransform parent;
  guint64 max_num;
  guint64 use_pts;
  guint64 result_limit;
  guint64 pre_ind;
  guint64 first_timestamp;
  guint64 pre_timestamp;
  guint64 num_frame;
  gdouble avg_rate;
  gdouble avg_pt;
  gchar* host;
  ParseData* data;
  GstSegment segment;
};

struct _GstAIBOX_DIST_ParseClass
{
  GstBaseTransformClass parentclass;
};

GType gst_aibox_dist_parse_get_type (void);

G_END_DECLS

#endif
