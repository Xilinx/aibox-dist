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
#  include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <gst/gst.h>
#include <tuple>

#include "gstaibox_dist_sum.hpp"
#include "cross_track.hpp"
#include <vitis/ai/profiling.hpp>
#include "debug.hpp"

GST_DEBUG_CATEGORY_STATIC (gst_aibox_dist_sum_debug);
#define GST_CAT_DEFAULT gst_aibox_dist_sum_debug

GQuark _scale_quark;

enum
{
  PROP_0,
  PROP_SYNC,
  PROP_SYNCTYPE,
  PROP_TOL,
  PROP_TIME_TOL,
  PROP_DELAY,
  PROP_TIMEOUT,
  PROP_PUSH_ALL_POP,
  PROP_USE_PTS,
  PROP_CONFIG,
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */

#define SINK_PAD_TEMP_NAME "sink_%u"
static GstStaticPadTemplate slave_sink_factory =
GST_STATIC_PAD_TEMPLATE (SINK_PAD_TEMP_NAME,
    GST_PAD_SINK,
    GST_PAD_REQUEST,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE (GST_VIDEO_FORMATS_ALL))
    );

/* src pad templates */
#define SRC_PAD_TEMP_NAME "src_%u"
static GstStaticPadTemplate slave_src_factory =
GST_STATIC_PAD_TEMPLATE (SRC_PAD_TEMP_NAME,
    GST_PAD_SRC,
    GST_PAD_SOMETIMES,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE (GST_VIDEO_FORMATS_ALL))
    );


#define gst_aibox_dist_sum_parent_class parent_class
G_DEFINE_TYPE (GstAIBox_dist_Sum, gst_aibox_dist_sum, GST_TYPE_ELEMENT);

G_DEFINE_TYPE (GstAIBox_dist_SumPad, gst_aibox_dist_sum_pad,
    GST_TYPE_PAD);


static GstPad *gst_aibox_dist_sum_request_new_pad (GstElement * element,
    GstPadTemplate * sink_templ, const gchar * name, const GstCaps * caps);

static void
gst_aibox_dist_sum_release_pad (GstElement * element, GstPad * pad);

static GstStateChangeReturn
gst_aibox_dist_sum_change_state (GstElement * element,
    GstStateChange transition);

static void gst_aibox_dist_sum_finalize (GObject * obj);
static GstFlowReturn
gst_aibox_dist_sum_collected (GstCollectPads * pads, gpointer user_data);
GstFlowReturn aibox_dist_sum_combined_return (GstAIBox_dist_Sum * self);

static void
gst_aibox_dist_sum_pad_class_init (GstAIBox_dist_SumPadClass * klass)
{
}

static void
gst_aibox_dist_sum_pad_init (GstAIBox_dist_SumPad * pad)
{
  pad->collect = NULL;
  pad->srcpad = NULL;
  pad->width = 0;
  pad->height = 0;
  pad->duration = GST_CLOCK_TIME_NONE;
  pad->curr_pts = GST_CLOCK_TIME_NONE;
  pad->eos_received = FALSE;
}

static void gst_aibox_dist_sum_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_aibox_dist_sum_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_aibox_dist_sum_sink_event (GstCollectPads * pads,
    GstCollectData * cdata, GstEvent * event, GstAIBox_dist_Sum * self);

/* GObject vmethod implementations */

/* initialize the aibox_dist_sum's class */
static void
gst_aibox_dist_sum_class_init (GstAIBox_dist_SumClass * klass)
{
  GObjectClass *gobject_class;
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);

  gobject_class = (GObjectClass *) klass;
  gstelement_class = (GstElementClass *) klass;

  gobject_class->set_property = gst_aibox_dist_sum_set_property;
  gobject_class->get_property = gst_aibox_dist_sum_get_property;
  gobject_class->finalize = gst_aibox_dist_sum_finalize;

  gst_element_class_set_details_simple (gstelement_class,
      "Plugin to sum up multi channel to do track",
      "Filter/Effect/Video",
      "Scale Meta data as per the resolution",
      "Xilinx Inc <www.xilinx.com>");

  gstelement_class->request_new_pad =
      GST_DEBUG_FUNCPTR (gst_aibox_dist_sum_request_new_pad);
  gstelement_class->release_pad =
      GST_DEBUG_FUNCPTR (gst_aibox_dist_sum_release_pad);
  gstelement_class->change_state =
      GST_DEBUG_FUNCPTR (gst_aibox_dist_sum_change_state);

  /* SINK pad template */
  gst_element_class_add_static_pad_template (gstelement_class,
      &slave_sink_factory);

  /* SRC pad template */
  gst_element_class_add_static_pad_template (gstelement_class,
      &slave_src_factory);

  g_object_class_install_property (gobject_class, PROP_SYNC,
      g_param_spec_boolean ("sync", "Sync",
          "Sync buffers on slave pads with buffers on master pad", TRUE,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_SYNCTYPE,
      g_param_spec_uint("sync-type", "Sync Type",
          "Sync type", 0, 2, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_TOL,
      g_param_spec_uint64 ("tol", "tolerance for sync",
          "tolerance of index diff for tracking take place", 0, G_MAXUINT64, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_TIME_TOL,
      g_param_spec_uint64 ("tol-time", "timestamp (ms) tolerance for sync",
          "tolerance of timestamp diff (ms) for tracking take place", 0, G_MAXUINT64, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_DELAY,
      g_param_spec_uint64 ("delay", "tolerance for delay",
          "tolerance of index diff for tracking take place", 0, G_MAXUINT64, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_TIMEOUT,
      g_param_spec_int64 ("timeout", "Timeout for collect pads",
          "Timeout in millisec \
          (-1) Disable timeout \
          Increase the timeout if debug logs are enabled", -1, G_MAXINT64, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_PUSH_ALL_POP,
      g_param_spec_int64 ("push-all-pop", "push all popped buffer",
          "Control if push all popped buffer", 0, G_MAXINT64, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_USE_PTS,
      g_param_spec_int64 ("use-pts", "Use pts for time sync",
          "Use pts for time sync", 0, G_MAXINT64, 0,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_CONFIG,
      g_param_spec_string ("config", "Config Json File Location",
          "Location of the config json file to read", NULL,
          GParamFlags(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_MUTABLE_READY)));

  _scale_quark = gst_video_meta_transform_scale_get_quark ();
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_aibox_dist_sum_init (GstAIBox_dist_Sum * self)
{
  int i;

  for (i = 0; i < MAX_SLAVE_SOURCES; i++) {
    self->sink_slave[i] = NULL;
    self->indAtSyncedTime[i] = 0;
    self->lastPopState[i] = 0;
  }

  self->num_slaves = 0;
  self->collect = gst_collect_pads_new ();
  self->sync = TRUE;
  self->tol = 0;
  self->time_tol = 50;
  self->timeSynced = FALSE;
  self->timeSyncedNeedSet = FALSE;
  self->syncType = AIBOX_DIST_SUM_SYNC_IND;
  self->flowcombiner = gst_flow_combiner_new ();
  self->prev_m_end_ts = GST_CLOCK_TIME_NONE;
  self->prev_meta_buf = NULL;
  self->timeout_thread = NULL;
  self->retry_timeout = 2000;
  self->push_all_pop = 0;
  self->push_test = true;
  self->timeout_issued = FALSE;
  self->start_thread = FALSE;
  self->stop_thread = FALSE;
  g_cond_init (&self->timeout_cond);
  g_mutex_init (&self->timeout_lock);
  g_mutex_init (&self->collected_lock);


  gst_collect_pads_set_function (self->collect, gst_aibox_dist_sum_collected,
      self);

  gst_collect_pads_set_event_function (self->collect,
      (GstCollectPadsEventFunction) gst_aibox_dist_sum_sink_event, self);
  self->system_inited = false;
  self->system_init_err = false;
  self->use_pts = 0;
  self->configPath = "/opt/xilinx/kv260-aibox-dist/share/vvas/cam_setup.json";
}

static void
gst_aibox_dist_sum_set_property (GObject * object,
    guint prop_id, const GValue * value, GParamSpec * pspec)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (object);

  switch (prop_id) {
    case PROP_SYNC:
      self->sync = g_value_get_boolean (value);
      break;
    case PROP_SYNCTYPE:
      self->syncType = (AIBOX_DIST_SUM_SYNC_TYPE)g_value_get_uint (value);
      break;
    case PROP_TIME_TOL:
      self->time_tol = g_value_get_uint64 (value);
      break;
    case PROP_DELAY:
      self->delay = g_value_get_uint64 (value);
      break;
    case PROP_TOL:
      self->tol = g_value_get_uint64 (value);
      break;
    case PROP_TIMEOUT:
      self->retry_timeout = g_value_get_int64 (value);
      break;
    case PROP_PUSH_ALL_POP:
      self->push_all_pop = g_value_get_int64 (value);
      break;
    case PROP_USE_PTS:
      self->use_pts = g_value_get_int64 (value);
      break;
    case PROP_CONFIG:
      self->configPath = std::string(g_value_get_string (value));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_aibox_dist_sum_get_property (GObject * object,
    guint prop_id, GValue * value, GParamSpec * pspec)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (object);

  switch (prop_id) {
    case PROP_SYNC:
      g_value_set_boolean (value, self->sync);
      break;
    case PROP_SYNCTYPE:
      g_value_set_uint (value, self->syncType);
      break;
    case PROP_TOL:
      g_value_set_uint64 (value, self->tol);
      break;
    case PROP_TIME_TOL:
      g_value_set_uint64 (value, self->time_tol);
      break;
    case PROP_DELAY:
      g_value_set_uint64 (value, self->delay);
      break;
    case PROP_TIMEOUT:
      g_value_set_int64 (value, self->retry_timeout);
      break;
    case PROP_PUSH_ALL_POP:
      g_value_set_int64 (value, self->push_all_pop);
      break;
    case PROP_USE_PTS:
      g_value_set_int64 (value, self->use_pts);
      break;
    case PROP_CONFIG:
      g_value_set_string (value, self->configPath.c_str());
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static gboolean
gst_aibox_dist_sum_sink_event (GstCollectPads * pads,
    GstCollectData * cdata, GstEvent * event, GstAIBox_dist_Sum * self)
{
  GstAIBox_dist_SumPad *pad = GST_AIBOX_DIST_SUM_PAD (cdata->pad);
  gboolean discard = FALSE;
  GstSegment segment;

  GST_DEBUG_OBJECT (pad, "Pad %s Got %s event: %" GST_PTR_FORMAT,
      GST_PAD_NAME (pad), GST_EVENT_TYPE_NAME (event), event);

  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_CAPS:{
      GstCaps *caps;
      gint fps_n;
      gint fps_d;

      gst_event_parse_caps (event, &caps);

      GST_INFO_OBJECT (pad, "Received caps %" GST_PTR_FORMAT, caps);

      if (!gst_video_info_from_caps (&pad->vinfo, caps)) {
        return FALSE;
      }

      /* handling dynamic resolution change */
      if (pad->width != GST_VIDEO_INFO_WIDTH (&pad->vinfo) ||
          pad->height != GST_VIDEO_INFO_HEIGHT (&pad->vinfo)) {
        GST_DEBUG_OBJECT (pad, "AIBox_dist_sum pad %s input resolution"
            " changed:Old: w=%d h=%d",
            GST_PAD_NAME (pad), pad->width, pad->height);
      }

      pad->width = GST_VIDEO_INFO_WIDTH (&pad->vinfo);
      pad->height = GST_VIDEO_INFO_HEIGHT (&pad->vinfo);

      fps_d = GST_VIDEO_INFO_FPS_D (&pad->vinfo);
      fps_n = GST_VIDEO_INFO_FPS_N (&pad->vinfo);

      /* duration in nano sec. */
      pad->duration = (fps_d * 1000000000) / fps_n;

      pad->curr_pts = 0;

      GST_INFO_OBJECT (pad, "aibox_dist_sum pad %s resolution w=%d h=%d",
          GST_PAD_NAME (pad), pad->width, pad->height);

      GST_INFO_OBJECT (pad, "aibox_dist_sum pad %s fps_n=%d fps_d=%d",
          GST_PAD_NAME (pad), fps_n, fps_d);

      GST_INFO_OBJECT (pad, "Duration pad %" GST_TIME_FORMAT,
          GST_TIME_ARGS (pad->duration));

      GST_DEBUG_OBJECT (pad, "Input Format No:%d   Name %s",
          GST_VIDEO_INFO_FORMAT (&pad->vinfo),
          GST_VIDEO_INFO_NAME (&pad->vinfo));

      /* set the same caps on the src pad */
      gst_pad_set_caps (pad->srcpad, caps);

      break;
    }
    case GST_EVENT_EOS:
      GST_DEBUG_OBJECT (pad, "received eos");
      pad->eos_received = TRUE;
      break;
    case GST_EVENT_SEGMENT:
      gst_event_copy_segment (event, &segment);

      if (!gst_pad_push_event (pad->srcpad, gst_event_new_segment(&segment))) {
        GST_WARNING_OBJECT (pad, "Pushing segment event to src pad failed");
      } else {
        GST_DEBUG_OBJECT (pad, "Pushing segment event to src pad is successful");
      }
      break;
    
    case GST_EVENT_STREAM_START:
      if (!gst_pad_push_event (pad->srcpad, event)) {
        GST_WARNING_OBJECT (pad, "Pushing stream start event to src pad failed");
        return FALSE;
      } else {
        GST_DEBUG_OBJECT (pad, "Pushing stream start event to src pad is successful");
        return TRUE;
      }
      break;

    default:
      break;
  }

  return gst_collect_pads_event_default (pads, cdata, event, discard);
}

static void
gst_aibox_dist_sum_finalize (GObject * obj)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (obj);
  gst_flow_combiner_free (self->flowcombiner);
  if (self->timeout_thread)
    g_thread_join(self->timeout_thread);
  if (self->prev_meta_buf)
    gst_buffer_unref (self->prev_meta_buf);
  if (self->system_inited && self->tracker) {
    delete self->tracker;
    self->system_inited = false;
  }
  gst_object_unref (self->collect);
}

static GstIterator *
gst_aibox_dist_sum_iterate_internal_links (GstPad * pad, GstObject * parent)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (parent);
  GValue val = { 0, };
  GstIterator *it = NULL;
  GstPad *other_pad;
  gboolean bFound = FALSE;

  GST_LOG_OBJECT (self, "Internal links for pad %s", GST_PAD_NAME (pad));

  GST_OBJECT_LOCK (parent);

  if (GST_PAD_IS_SINK (pad)) {
    GstAIBox_dist_SumPad *sink_pad = (GstAIBox_dist_SumPad *) pad;
    other_pad = sink_pad->srcpad;
  } else {
    {
      int i;
      for (i = 0; i < MAX_SLAVE_SOURCES; i++) {
        if (self->sink_slave[i] && self->sink_slave[i]->srcpad == pad) {
          other_pad = GST_PAD (self->sink_slave[i]);
          bFound = TRUE;
          break;
        }
      }
    }

    if (bFound == FALSE) {
      GST_ERROR_OBJECT (self, "wrong pad, %s, in iterate internal link",
          GST_PAD_NAME (pad));
      GST_OBJECT_UNLOCK (parent);
      return NULL;
    }
  }

  g_value_init (&val, GST_TYPE_PAD);
  g_value_set_object (&val, other_pad);
  it = gst_iterator_new_single (GST_TYPE_PAD, &val);
  g_value_unset (&val);
  GST_OBJECT_UNLOCK (parent);

  return it;
}

static GstPad *
gst_aibox_dist_sum_request_new_pad (GstElement * element,
    GstPadTemplate * sink_templ, const gchar * name, const GstCaps * caps)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (element);
  GstAIBox_dist_SumClass *klass = GST_AIBOX_DIST_SUM_GET_CLASS (self);
  const gchar *sink_tmpl_name = GST_PAD_TEMPLATE_NAME_TEMPLATE (sink_templ);
  GstAIBox_dist_SumPad *sink_pad = NULL;
  GstPad *src_pad = NULL;
  GstAIBox_dist_SumCollectData *collect_data = NULL;
  GstPadTemplate *src_templ;
  gboolean bret = TRUE;
  unsigned int slave_index = 0xFFFFFFFF;

  GST_DEBUG_OBJECT (self, "Requesting pad with name %s", name);

  if (!g_strcmp0 (sink_tmpl_name, SINK_PAD_TEMP_NAME)) {
    /* Get the index of the slave */
    if (name && sscanf (name, SINK_PAD_TEMP_NAME, &slave_index) == 1) {
      if (slave_index >= MAX_SLAVE_SOURCES) {
        GST_ERROR_OBJECT (self, "Max allowed sources, %d, already attached",
            MAX_SLAVE_SOURCES);
        return NULL;
      } else if (slave_index > self->num_slaves) {
        GST_ERROR_OBJECT (self, "Pad index must be incremental"
            "should be %d, but received %d", self->num_slaves, slave_index);
        return NULL;
      }
    } else {
      GST_ERROR_OBJECT (self, "Requested Sink pad name, %s, format is not "
          "correct", name);
      return NULL;
    }

    sink_pad = self->sink_slave[slave_index];
  } else {
    GST_ERROR_OBJECT (self, "sink pad template name %s, not matching "
        "with any sink pad template", sink_tmpl_name);
    goto error;
  }

  /* Check if pad already exists */
  if (sink_pad) {
    GST_ERROR_OBJECT (self, "pad %s already exist", GST_PAD_NAME (sink_pad));
    goto error;
  }

  /* requested pad with name doesn't exists. Create new pad */

  sink_pad =
      (GstAIBox_dist_SumPad*)g_object_new (GST_TYPE_AIBOX_DIST_SUM_PAD, "name", name, "direction",
      sink_templ->direction, "template", sink_templ, NULL);

  collect_data = (GstAIBox_dist_SumCollectData *)
      gst_collect_pads_add_pad (self->collect,
      GST_PAD (sink_pad), sizeof (GstAIBox_dist_SumCollectData), NULL, TRUE);

  collect_data->sinkpad = sink_pad;
  sink_pad->collect = collect_data;

  bret = gst_element_add_pad (element, GST_PAD (sink_pad));

  if (!bret) {
    GST_ERROR_OBJECT (self, "Failed to add sink pad %s", name);
    goto error;
  }

  gst_pad_set_iterate_internal_links_function (GST_PAD (sink_pad),
      gst_aibox_dist_sum_iterate_internal_links);

  GST_PAD_SET_PROXY_CAPS (GST_PAD (sink_pad));
  GST_PAD_SET_PROXY_ALLOCATION (GST_PAD (sink_pad));
  GST_PAD_SET_PROXY_SCHEDULING (GST_PAD (sink_pad));

  /* create source pad corresponding to sinkpad */

  if (!g_strcmp0 (sink_tmpl_name, SINK_PAD_TEMP_NAME)) {
    char src_slave_name[25];
    self->sink_slave[slave_index] = sink_pad;
    src_templ = gst_element_class_get_pad_template (GST_ELEMENT_CLASS (klass),
        SRC_PAD_TEMP_NAME);

    if (src_templ == NULL) {
      GST_ERROR_OBJECT (self, "For sink slave %s, src template is NULL",
          GST_PAD_NAME (sink_pad));
      goto error;
    }

    sprintf (src_slave_name, SRC_PAD_TEMP_NAME, slave_index);

    src_pad = gst_pad_new_from_template (src_templ, src_slave_name);

    if (src_pad) {
      GST_INFO_OBJECT (self, "Created %s pad", GST_PAD_NAME (src_pad));
    } else {
      GST_ERROR_OBJECT (self, "Unable to create %s pad",
          GST_PAD_NAME (sink_pad));
      goto error;
    }
    self->num_slaves++;
  }

  gst_element_add_pad ((GstElement *) self, src_pad);
  gst_flow_combiner_add_pad (self->flowcombiner, src_pad);

  sink_pad->srcpad = src_pad;
  sink_pad->fret = GST_FLOW_OK;
  sink_pad->sent_eos = false;
  sink_pad->stream_state = AIBOX_DIST_SUM_STATE_FIRST_BUFFER;

  gst_pad_set_iterate_internal_links_function (src_pad,
      gst_aibox_dist_sum_iterate_internal_links);

  return GST_PAD (sink_pad);

error:
  if (collect_data)
    free (collect_data);
  if (sink_pad)
    g_free (sink_pad);

  return NULL;
}

static void
gst_aibox_dist_sum_release_pad (GstElement * element, GstPad * pad)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (element);
  GstAIBox_dist_SumPad *sink_pad = GST_AIBOX_DIST_SUM_PAD (pad);

  GST_INFO_OBJECT (self, "  %" GST_PTR_FORMAT, pad);

  gst_collect_pads_remove_pad (self->collect, pad);

  if (sink_pad && sink_pad->srcpad) {
    gst_element_remove_pad (element, sink_pad->srcpad);
    gst_flow_combiner_remove_pad (self->flowcombiner, sink_pad->srcpad);
  }

  gst_element_remove_pad (element, pad);
}


GstFlowReturn
aibox_dist_sum_combined_return (GstAIBox_dist_Sum * self)
{
  GstFlowReturn fret = GST_FLOW_OK;
  int slave_idx;

  for (slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
    GstAIBox_dist_SumPad *sink_slave = self->sink_slave[slave_idx];
    fret = gst_flow_combiner_update_pad_flow (self->flowcombiner, sink_slave->srcpad,
        sink_slave->fret);
  }
  GST_LOG_OBJECT (self, "combined to return %s", gst_flow_get_name (fret));
  return fret;
}

static GstFlowReturn
aibox_dist_sum_get_min_end_ts (GstAIBox_dist_Sum * self,
    GstCollectPads * pads, GstClockTime * min_end_ts)
{
  GstBuffer *buffer = NULL;
  GstClockTime cur_start_ts = GST_CLOCK_TIME_NONE;
  GstClockTime cur_end_ts = GST_CLOCK_TIME_NONE;
  GstClockTime cur_dur = GST_CLOCK_TIME_NONE;
  guint slave_idx;

  /* find minimum end ts of buffer at each slave sink pad */
  for (slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
    GstAIBox_dist_SumPad *sink_slave = self->sink_slave[slave_idx];

    buffer = gst_collect_pads_peek (pads, (GstCollectData *) sink_slave->collect);
    if (!buffer) {
      if (GST_COLLECT_PADS_STATE_IS_SET ((GstCollectData *) sink_slave->collect,
              GST_COLLECT_PADS_STATE_EOS)) {
        if (!sink_slave->sent_eos) {
          GST_INFO_OBJECT (self, "pushing eos on pad %s",
              GST_PAD_NAME (sink_slave->srcpad));

          if (!gst_pad_push_event (sink_slave->srcpad, gst_event_new_eos ()))
            GST_ERROR_OBJECT (self, "failed to send eos event on pad %s",
                GST_PAD_NAME (sink_slave->srcpad));

          sink_slave->sent_eos = TRUE;
        }
        sink_slave->fret = GST_FLOW_EOS;
        continue;
      } else {
        GST_ERROR_OBJECT (self, "buffer is not availale at slave pad index %d",
            slave_idx);
        sink_slave->fret = GST_FLOW_ERROR;
        goto exit;
      }
    }

    cur_start_ts = GST_BUFFER_PTS (buffer);
    cur_dur = GST_BUFFER_DURATION (buffer);
    gst_buffer_unref (buffer);

    if (!GST_CLOCK_TIME_IS_VALID (cur_dur)
        && GST_CLOCK_TIME_IS_VALID (sink_slave->duration)) {
      GST_LOG_OBJECT (self,
          "slave index %d does not have a valid timestamp, calculated duration from framerate %"
          GST_TIME_FORMAT, slave_idx, GST_TIME_ARGS (sink_slave->duration));
      cur_dur = sink_slave->duration;
    }

    if (!GST_CLOCK_TIME_IS_VALID (cur_start_ts)
        || !GST_CLOCK_TIME_IS_VALID (cur_dur)) {
      GST_WARNING_OBJECT (self,
          "does not have valid frame start timestamp and/or duration to do sync");
      /* even one invalid timestamp can stop sync mode */
      self->sync = FALSE;
    } else {
      /* have valid start_ts and duration */
      cur_end_ts = cur_start_ts + cur_dur;

      if (cur_end_ts < *min_end_ts) {
        /*update min end ts */
        *min_end_ts = cur_end_ts;
      }
    }
  }

exit:
  GST_DEBUG_OBJECT (self, "found minimum end ts %" GST_TIME_FORMAT,
      GST_TIME_ARGS (*min_end_ts));

  return aibox_dist_sum_combined_return (self);
}

static uint64_t
aibox_dist_sum_get_time(GstAIBox_dist_Sum * self, GstBuffer* buf, GstInferenceMeta *infer_meta)
{
  uint64_t time = *(uint64_t *)infer_meta->prediction->reserved_4;
  if (self->use_pts)
  {
    time = GST_BUFFER_PTS(buf) / 1e6;
  }
  return time;
}

static std::tuple<uint64_t, uint64_t, uint64_t, uint64_t>
aibox_dist_sum_get_max_frame_id_from_collect(GstAIBox_dist_Sum * self, GstCollectPads * pads)
{
  uint64_t max_ind = 0, min_ind = 0, first_set = 0;
  uint64_t max_time = 0, min_time = 0;
  for (guint slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
    GstAIBox_dist_SumPad *sink_slave = self->sink_slave[slave_idx];
    GstBuffer *writable_buffer = NULL;
    GstBuffer *sbuffer = NULL;
    GstClockTime s_cur_start_ts = GST_CLOCK_TIME_NONE;
    GstClockTime s_cur_end_ts = GST_CLOCK_TIME_NONE;
    GstClockTime s_cur_dur = GST_CLOCK_TIME_NONE;

    GST_COLLECT_PADS_STREAM_LOCK (pads);
    sbuffer = gst_collect_pads_peek (pads, (GstCollectData *) sink_slave->collect);
    GST_COLLECT_PADS_STREAM_UNLOCK (pads);

    if (!sbuffer) {
      if (GST_COLLECT_PADS_STATE_IS_SET ((GstCollectData *) sink_slave->collect, GST_COLLECT_PADS_STATE_EOS)) {
        if (!sink_slave->sent_eos) {
          GST_INFO_OBJECT (self, "pushing eos on pad %s", GST_PAD_NAME (sink_slave->srcpad));
          if (!gst_pad_push_event (sink_slave->srcpad, gst_event_new_eos ()))
            GST_ERROR_OBJECT (self, "failed to send eos event on pad %s", GST_PAD_NAME (sink_slave->srcpad));
          sink_slave->sent_eos = TRUE;
        }
        sink_slave->fret = GST_FLOW_EOS;
        continue;
      } else {
        GST_ERROR_OBJECT (self, "failed to get buffer from collectpad %s", GST_PAD_NAME (sink_slave));
        sink_slave->fret = GST_FLOW_ERROR;
        continue;
      }
    }

    GstInferenceMeta *infer_meta = ((GstInferenceMeta *) gst_buffer_get_meta ((GstBuffer *)
          sbuffer, gst_inference_meta_api_get_type ()));

    if (!infer_meta) {
      GST_ERROR_OBJECT (self, "no meta in the buffer.");
    } else {
      if(infer_meta->prediction->reserved_3 != NULL) {
        struct cros_reid_info* meta = (struct cros_reid_info*)infer_meta->prediction->reserved_3;

        uint64_t time = aibox_dist_sum_get_time(self, sbuffer, infer_meta);
        GST_ERROR_OBJECT (self, "slave_idx: %u, ind: %lu, time: %lu, pts: %lu .", slave_idx, meta->frame_id, time, self->use_pts);

        uint64_t frameIDMinusSyncInd = meta->frame_id - self->indAtSyncedTime[slave_idx];
        if (first_set == 0) {
          max_ind = frameIDMinusSyncInd;
          min_ind = frameIDMinusSyncInd;
          max_time = time;
          min_time = time;
          first_set = 1;
        } else {
          if (frameIDMinusSyncInd > max_ind) {
            max_ind = frameIDMinusSyncInd;
          }
          if (frameIDMinusSyncInd < min_ind) {
            min_ind = frameIDMinusSyncInd;
          }
          if (time > max_time) {
            max_time = time;
          }
          if (time < min_time) {
            min_time = time;
          }
        }
      }
      gst_buffer_unref (sbuffer);
    }
  }
  return std::make_tuple(max_ind, max_ind - min_ind, max_time, max_time - min_time);
}

static GstFlowReturn
aibox_dist_sum_process (GstAIBox_dist_Sum * self, GstCollectPads * pads,
    GstClockTime min_end_ts)
{
  __TIC__(SUM_PROCESS);
  GstBuffer *mbuffer = NULL;
  guint slave_idx;
  GstMeta *infer_meta = NULL;
  GstClockTime m_cur_start_ts = GST_CLOCK_TIME_NONE;
  GstClockTime m_cur_end_ts = GST_CLOCK_TIME_NONE;
  GstClockTime m_cur_dur = GST_CLOCK_TIME_NONE;
  vector<vector<TrackerResult>> result;
  GST_DEBUG_OBJECT (self, "sum process start.");

// get max timestamp of the buffers;
  if (!self->system_inited) {
    self->tracker = new CrossTracker(self->configPath, self->num_slaves, 1920, 1080);
    if (!self->tracker->ConstrIsOK()) {
      GError* error = g_error_new (GST_LIBRARY_ERROR, GST_LIBRARY_ERROR_INIT,
          "Tracker failed to initialize.");
      GstMessage* message = gst_message_new_error (GST_OBJECT (self), error, "Plugin aibox_dist_sum: Tracker failed to initialize.");
      gst_element_post_message (GST_ELEMENT (self), message);

      g_error_free (error);
      self->system_init_err = true;
    }
    self->system_inited = true;
  }

  bool allArrive = true;
  std::vector<guint> notArriveId;
  for (slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
    GstAIBox_dist_SumPad *sink_slave = self->sink_slave[slave_idx];
    GstBuffer *sbuffer = NULL;
    GST_COLLECT_PADS_STREAM_LOCK (pads);
    sbuffer = gst_collect_pads_peek (pads, (GstCollectData *) sink_slave->collect);
    GST_COLLECT_PADS_STREAM_UNLOCK (pads);

    if (!sbuffer) {
      allArrive = false;
      notArriveId.push_back(slave_idx);
    } else {
      gst_buffer_unref(sbuffer);
    }
  }
  if (!allArrive) {
    std::ostringstream oss;
    for (auto id : notArriveId)
    {
      oss << id << ",";
    }
    GST_INFO_OBJECT (self, "not all arrived. %s", oss.str().c_str());
    return GST_FLOW_OK;
  }


  std::tuple<uint64_t,uint64_t,uint64_t,uint64_t> ret = aibox_dist_sum_get_max_frame_id_from_collect(self, pads); // functon call
  uint64_t maxind_ref = std::get<0>(ret);
  uint64_t indRange = std::get<1>(ret);
  uint64_t maxtime_ref= std::get<2>(ret);
  uint64_t timeRange = std::get<3>(ret);

  std::vector<cros_reid_info*> trackInput;
  std::vector<bool> popflags;
  std::vector<bool> pushflags;
  std::vector<uint64_t> delays;
  bool doTrack = true;

  struct timespec _t;
  clock_gettime(CLOCK_REALTIME, &_t);
  uint64_t curtime = _t.tv_sec*1000 + lround(_t.tv_nsec/1e6);
  if (self->use_pts) {
    curtime = (gst_clock_get_time(GST_ELEMENT_CLOCK(self)) - gst_element_get_base_time (GST_ELEMENT_CAST(self)))/1e6;
  }

  for (slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
    GstAIBox_dist_SumPad *sink_slave = self->sink_slave[slave_idx];
    GstBuffer *writable_buffer = NULL;
    GstBuffer *sbuffer = NULL;
    GstClockTime s_cur_start_ts = GST_CLOCK_TIME_NONE;
    GstClockTime s_cur_end_ts = GST_CLOCK_TIME_NONE;
    GstClockTime s_cur_dur = GST_CLOCK_TIME_NONE;

    GST_COLLECT_PADS_STREAM_LOCK (pads);
    sbuffer = gst_collect_pads_peek (pads, (GstCollectData *) sink_slave->collect);
    GST_COLLECT_PADS_STREAM_UNLOCK (pads);

    if (!sbuffer) {
      if (GST_COLLECT_PADS_STATE_IS_SET ((GstCollectData *) sink_slave->collect, GST_COLLECT_PADS_STATE_EOS)) {
        GST_INFO_OBJECT (self, "pad %s eos", GST_PAD_NAME (sink_slave->srcpad));
        if (!sink_slave->sent_eos) {
          GST_INFO_OBJECT (self, "pushing eos on pad %s", GST_PAD_NAME (sink_slave->srcpad));
          if (!gst_pad_push_event (sink_slave->srcpad, gst_event_new_eos ()))
            GST_ERROR_OBJECT (self, "failed to send eos event on pad %s", GST_PAD_NAME (sink_slave->srcpad));
          sink_slave->sent_eos = TRUE;
        }
        sink_slave->fret = GST_FLOW_EOS;
        popflags.push_back(false);
        pushflags.push_back(false);
        doTrack = false;
        continue;
      } else {
        GST_ERROR_OBJECT (self, "failed to get buffer from collectpad %s", GST_PAD_NAME (sink_slave));
        sink_slave->fret = GST_FLOW_ERROR;
        goto exit;
      }
    }

    GstInferenceMeta *infer_meta = ((GstInferenceMeta *) gst_buffer_get_meta ((GstBuffer *)
          sbuffer, gst_inference_meta_api_get_type ()));


    uint64_t oldIndAt = self->indAtSyncedTime[slave_idx];
    uint64_t frameId = -1;
    uint64_t time = -1;
    if (0) {
      popflags.push_back(true);
      pushflags.push_back(true);
    } else
    if (!infer_meta) {
      doTrack = false;
      popflags.push_back(true);
      pushflags.push_back(true);
    } else {
      if(infer_meta->prediction->reserved_3 != NULL) {
        struct cros_reid_info* meta = (struct cros_reid_info*)infer_meta->prediction->reserved_3;
        time = aibox_dist_sum_get_time(self, sbuffer, infer_meta);
        frameId = meta->frame_id;

        uint64_t tol = 0;
        uint64_t range = 0;
        uint64_t max_ref= 0;
        uint64_t curValue = 0;



        if ( (self->syncType == AIBOX_DIST_SUM_SYNC_TIME_IND && !self->timeSynced)
            || self->syncType == AIBOX_DIST_SUM_SYNC_TIME) {
          tol = self->time_tol;
          range = timeRange;
          max_ref = maxtime_ref;
          curValue = time;
        } else {
          tol = self->tol;
          range = indRange;
          max_ref = maxind_ref;
          curValue = meta->frame_id - self->indAtSyncedTime[slave_idx];
        }

        if (range <= tol) {
          popflags.push_back(true);
          pushflags.push_back(true);
          trackInput.push_back(meta);
          if (self->syncType != AIBOX_DIST_SUM_SYNC_IND && !self->timeSynced) {
            self->timeSyncedNeedSet = TRUE;
            self->indAtSyncedTime[slave_idx] = meta->frame_id;
          }
        } else  {
          doTrack = false;
          if ( max_ref - curValue > tol) {
            popflags.push_back(true);
            pushflags.push_back(false);
          } else {
            popflags.push_back(false);
            pushflags.push_back(false);
          }
        }
      } else {
        doTrack = false;
        popflags.push_back(true);
        pushflags.push_back(true);
      }
    }


    delays.push_back(curtime - time);

    gst_buffer_unref (sbuffer);
  }

  if (!doTrack && self->syncType != 0) {
    for (slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
      if (delays[slave_idx] > self->delay) {
        GST_INFO_OBJECT (self, "Cam: %d, pop for large delay %lu : %lu", slave_idx, delays[slave_idx], self->delay);
        popflags[slave_idx] = true;
        pushflags[slave_idx] = false;
      } else if (self->system_init_err) {
        GST_INFO_OBJECT (self, "Cam: %d, pop push for tracker init error.", slave_idx);
        popflags[slave_idx] = true;
        pushflags[slave_idx] = true;
      }
    }
  }

  if ( self->timeSyncedNeedSet) {
    self->timeSynced = TRUE;
    self->timeSyncedNeedSet = FALSE;
  }

  static int test_ind = 0;
  {
  vector<vector<TrackerResult>*> res;
  __TIC__(SUM_PROCESS_TRACK);
  if (doTrack) {
    test_ind++;
    try{
      __TIC__(TRACK)
      auto tmp = self->tracker->Run(trackInput);
      __TOC__(TRACK)
      __TIC__(NEW)
      int ch = 0;
      for (auto r : tmp)
      {
        res.push_back(new vector<TrackerResult>(r));
        ch++;
      }

      __TOC__(NEW)
    } catch (const cv::Exception& e) {
      GST_ERROR_OBJECT (self, "Track exception.");
      exit(1);
    }
  }
  __TOC__(SUM_PROCESS_TRACK);

  for (slave_idx = 0; slave_idx < self->num_slaves; slave_idx++) {
    GstAIBox_dist_SumPad *sink_slave = self->sink_slave[slave_idx];
    GstBuffer *pbuffer = NULL;
    GstInferenceMeta *infer_meta = NULL;
    if (popflags[slave_idx]) {
      GST_COLLECT_PADS_STREAM_LOCK (pads);
      GstBuffer *sbuffer = gst_collect_pads_pop (pads, (GstCollectData *) sink_slave->collect);
      GST_COLLECT_PADS_STREAM_UNLOCK (pads);
      pbuffer = gst_buffer_make_writable (sbuffer);
      infer_meta = ((GstInferenceMeta *) gst_buffer_get_meta ((GstBuffer *)
          pbuffer, gst_inference_meta_api_get_type ()));
      if (infer_meta && infer_meta->prediction && infer_meta->prediction->reserved_4) {
        delete (uint64_t*)infer_meta->prediction->reserved_4;
        infer_meta->prediction->reserved_4 = NULL;
      }

      if (pbuffer) {
        if (doTrack) {
          if (!infer_meta) {
            GST_ERROR_OBJECT (self, "no meta in the buffer.");
          } else {
            if(infer_meta->prediction->reserved_1 == NULL) {
              infer_meta->prediction->reserved_1 = (void*)(int64_t)(slave_idx);
            }
            if(infer_meta->prediction->reserved_5 == NULL) {
              infer_meta->prediction->reserved_5 = (void*)(res[slave_idx]);
            }
          }
        }

        if ( self->push_test || (self->push_all_pop > 0 || pushflags[slave_idx]) ) {
          self->push_test = false;
          sink_slave->fret = gst_pad_push (sink_slave->srcpad, pbuffer);
          GST_DEBUG_OBJECT (self, "Cam %d Push, ret: %d.", slave_idx, sink_slave->fret);
          if (sink_slave->fret != GST_FLOW_OK) {
            goto exit;
          }
        } else {
          if (infer_meta && infer_meta->prediction && infer_meta->prediction->reserved_3) {
            delete (struct cros_reid_info*)infer_meta->prediction->reserved_3;
            infer_meta->prediction->reserved_3 = NULL;
          }
          gst_buffer_unref (pbuffer);
        }
      }
    } else {
      assert(!doTrack && !pushflags[slave_idx]);
    }
  }
  }

exit:
  __TOC__(SUM_PROCESS);
  GST_DEBUG_OBJECT (self, "sum process stop.");

  return aibox_dist_sum_combined_return (self);
}

// TO CHECK
static GstFlowReturn
gst_aibox_dist_sum_collected (GstCollectPads * pads, gpointer user_data)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (user_data);
  GstClockTime min_end_ts = GST_CLOCK_TIME_NONE;
  GstFlowReturn fret = GST_FLOW_OK;

  g_mutex_lock (&self->collected_lock);
  fret = aibox_dist_sum_process (self, pads, min_end_ts);
  g_mutex_unlock (&self->collected_lock);

exit:
  return fret;
}

static gpointer
timeout_func (gpointer data)
{
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (data);
  GST_ERROR_OBJECT (self, "start timeout routine");
  gint64 end_time;
  GstFlowReturn fret;
  gboolean is_locked = FALSE;
  while (self->retry_timeout > 0) {
    g_mutex_lock (&self->timeout_lock);
    is_locked = TRUE;
    if (!self->start_thread) {
      end_time = G_MAXINT64;
    }
    else {
      end_time = g_get_monotonic_time () + self->retry_timeout;
    }

    if (!g_cond_wait_until (&self->timeout_cond, &self->timeout_lock, end_time)) {
      GST_INFO_OBJECT (self, "Timeout occured");
      self->timeout_issued = TRUE;
      if (is_locked) {
        g_mutex_unlock (&self->timeout_lock);
        is_locked = FALSE;
      }
      GST_ERROR_OBJECT (self, "collect from timeout");
      fret = gst_aibox_dist_sum_collected (self->collect, self);
      g_mutex_lock (&self->timeout_lock);
      is_locked = TRUE;
      self->timeout_issued = FALSE;
      if (fret != GST_FLOW_OK) {
        if (is_locked) {
          g_mutex_unlock (&self->timeout_lock);
          is_locked = FALSE;
        }
        GST_ERROR_OBJECT (self, "collect from timeout ERROR, quit timeout");
        //break;
      }
    }
    else if (self->stop_thread) {
      if (is_locked) {
        g_mutex_unlock (&self->timeout_lock);
        is_locked = FALSE;
      }
      break;
    }

    if (is_locked) {
      g_mutex_unlock (&self->timeout_lock);
      is_locked = FALSE;
    }
  }
  GST_ERROR_OBJECT (self, "quit timeout routine");
  return NULL;
}

static GstStateChangeReturn
gst_aibox_dist_sum_change_state (GstElement * element,
    GstStateChange transition)
{
  GstStateChangeReturn ret;
  GstAIBox_dist_Sum *self = GST_AIBOX_DIST_SUM (element);
  GstState pending;
  GstState state;

//  ret = gst_element_get_state (element,
//  &state, 
//  &pending, GST_CLOCK_TIME_NONE);
  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY:
      break;
    case GST_STATE_CHANGE_READY_TO_PAUSED:
      gst_collect_pads_start (self->collect);
      if (self->retry_timeout != -1) {
        g_mutex_lock (&self->timeout_lock);
        self->retry_timeout *= G_TIME_SPAN_MILLISECOND;
        g_mutex_unlock (&self->timeout_lock);
        /* start thread to monitor buffer flow */
        self->timeout_thread = g_thread_new ("aibox_dist_sum watchdog", timeout_func, self);
      }
      break;
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
      if (self->retry_timeout != -1) {
        g_mutex_lock (&self->timeout_lock);
        self->start_thread = TRUE;
        g_cond_signal (&self->timeout_cond);
        g_mutex_unlock (&self->timeout_lock);
      }
      break;
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
      if (self->retry_timeout != -1) {
        g_mutex_lock (&self->timeout_lock);
        self->start_thread = FALSE;
        g_mutex_unlock (&self->timeout_lock);
      }
      break;
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      if (self->retry_timeout != -1) {
        g_mutex_lock (&self->timeout_lock);
        self->stop_thread = TRUE;
        g_cond_signal (&self->timeout_cond);
        g_mutex_unlock (&self->timeout_lock);
        if (self->timeout_thread) {
          GST_LOG_OBJECT (self, "waiting for watchdog thread to join");
          g_thread_join (self->timeout_thread);
          self->timeout_thread = NULL;
        }
        self->retry_timeout /= G_TIME_SPAN_MILLISECOND;
        self->timeout_issued = FALSE;
        self->stop_thread = FALSE;
      }
      gst_collect_pads_stop (self->collect);
      break;
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (parent_class)->change_state (element, transition);

  switch (transition) {
    case GST_STATE_CHANGE_READY_TO_NULL:
      break;
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (parent_class)->change_state (element, transition);

  return ret;
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
aibox_dist_sum_init (GstPlugin * aibox_dist_sum)
{
  /* debug category for fltering log messages
   * exchange the string 'Template aibox_dist_sum' with your description
   */
  GST_DEBUG_CATEGORY_INIT (gst_aibox_dist_sum_debug, "aibox_dist_sum",
      0, "Template aibox_dist_sum");

  return gst_element_register (aibox_dist_sum, "aibox_dist_sum",
      GST_RANK_NONE, GST_TYPE_AIBOX_DIST_SUM);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "aibox_dist_sum"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    aibox_dist_sum,
    "Xilinx plugin to do Cross Stream Track for distributed AIBox",
    aibox_dist_sum_init, "1.0", "MIT/X11",
    "Xilinx SOM AIBOX plugin", "http://xilinx.com/")
