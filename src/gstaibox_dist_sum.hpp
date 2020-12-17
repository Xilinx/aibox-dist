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

#ifndef __GST_AIBOX_DIST_SUM_H__
#define __GST_AIBOX_DIST_SUM_H__

#include <gst/gst.h>
#include <gst/base/gstcollectpads.h>
#include <gst/video/video.h>
#include <gst/base/gstflowcombiner.h>
#include <gst/vvas/gstinferencemeta.h>
#include "cross_track.hpp"

G_BEGIN_DECLS
/* #defines don't like whitespacey bits */
#define GST_TYPE_AIBOX_DIST_SUM \
  (gst_aibox_dist_sum_get_type())
#define GST_AIBOX_DIST_SUM(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_AIBOX_DIST_SUM,GstAIBox_dist_Sum))
#define GST_AIBOX_DIST_SUM_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_AIBOX_DIST_SUM,GstAIBox_dist_SumClass))
#define GST_IS_AIBOX_DIST_SUM(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_AIBOX_DIST_SUM))
#define GST_IS_AIBOX_DIST_SUM_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_AIBOX_DIST_SUM))
#define GST_AIBOX_DIST_SUM_GET_CLASS(obj) \
  (G_TYPE_INSTANCE_GET_CLASS ((obj), GST_TYPE_AIBOX_DIST_SUM, GstAIBox_dist_SumClass))
#define GST_TYPE_AIBOX_DIST_SUM_PAD (gst_aibox_dist_sum_pad_get_type())
#define GST_AIBOX_DIST_SUM_PAD(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_AIBOX_DIST_SUM_PAD, GstAIBox_dist_SumPad))
#define GST_AIBOX_DIST_SUM_PAD_CLASS(klass) (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_VIDEO_MIXER_PAD, GstAIBox_dist_SumPadClass))
#define GST_IS_AIBOX_DIST_SUM_PAD(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_AIBOX_DIST_SUM_PAD))
#define GST_IS_AIBOX_DIST_SUM_PAD_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_AIBOX_DIST_SUM_PAD))
typedef struct _GstAIBox_dist_Sum GstAIBox_dist_Sum;
typedef struct _GstAIBox_dist_SumClass GstAIBox_dist_SumClass;
typedef struct _GstAIBox_dist_SumPad GstAIBox_dist_SumPad;
typedef struct _GstAIBox_dist_SumPadClass GstAIBox_dist_SumPadClass;
typedef struct _GstAIBox_dist_SumCollectData GstAIBox_dist_SumCollectData;

typedef enum {
  AIBOX_DIST_SUM_STATE_FIRST_BUFFER,
  AIBOX_DIST_SUM_STATE_DROP_BUFFER,
  AIBOX_DIST_SUM_STATE_PROCESS_BUFFER,
} AIBOX_DIST_SUM_STREAM_STATE;

struct _GstAIBox_dist_SumCollectData
{
  GstCollectData collectdata;   /* we extend the CollectData */
  GstAIBox_dist_SumPad *sinkpad;
};

struct _GstAIBox_dist_SumPad
{
  GstPad parent;
  GstAIBox_dist_SumCollectData *collect;
  GstPad *srcpad;
  guint width;
  guint height;
  GstClockTime duration;
  GstClockTime curr_pts;
  gboolean eos_received;
  GstVideoInfo vinfo;
  GstFlowReturn fret;
  gboolean sent_eos;
  AIBOX_DIST_SUM_STREAM_STATE stream_state;
};

#define MAX_SLAVE_SOURCES 4

typedef enum {
  AIBOX_DIST_SUM_SYNC_IND,
  AIBOX_DIST_SUM_SYNC_TIME,
  AIBOX_DIST_SUM_SYNC_TIME_IND,
} AIBOX_DIST_SUM_SYNC_TYPE;

struct _GstAIBox_dist_Sum
{
  GstElement element;
  GstCollectPads *collect;
  GstAIBox_dist_SumPad *sink_slave[MAX_SLAVE_SOURCES];
  GstFlowCombiner *flowcombiner;
  guint num_slaves;
  guint system_inited;
  guint system_init_err;
  gboolean sync;
  GstClockTime prev_m_end_ts;
  GstBuffer *prev_meta_buf; /* buffer holds metadata only but not data */
  GThread *timeout_thread;
  gint64 retry_timeout;
  gint64 push_all_pop;
  gint64 use_pts;
  std::string configPath;
  gboolean push_test;
  guint64 time_tol;
  guint64 delay;
  guint64 tol;
  AIBOX_DIST_SUM_SYNC_TYPE syncType;
  guint64  indAtSyncedTime[MAX_SLAVE_SOURCES];
  guint64  lastPopState[MAX_SLAVE_SOURCES];
  gboolean timeSynced;
  gboolean timeSyncedNeedSet;
  gboolean timeout_issued;
  gboolean start_thread;
  gboolean stop_thread;
  GCond timeout_cond;
  GMutex timeout_lock;
  GMutex collected_lock;
  CrossTracker *tracker;
};

struct _GstAIBox_dist_SumPadClass
{
  GstPadClass parent_class;
};

struct _GstAIBox_dist_SumClass
{
  GstElementClass parent_class;
};

GType gst_aibox_dist_sum_get_type (void);
GType gst_aibox_dist_sum_pad_get_type (void);

G_END_DECLS
#endif /* __GST_AIBOX_DIST_SUM_H__ */
