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

#ifndef _GST_AIBOX_DIST_ATTACH_H_
#define _GST_AIBOX_DIST_ATTACH_H_

#include <gst/base/gstbasetransform.h>
#include <zmq.hpp>

G_BEGIN_DECLS

#define GST_TYPE_AIBOX_DIST_ATTACH   (gst_aibox_dist_attach_get_type())
#define GST_AIBOX_DIST_ATTACH(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_AIBOX_DIST_ATTACH,GstAIBOX_DIST_Attach))
#define GST_AIBOX_DIST_ATTACH_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_AIBOX_DIST_ATTACH,GstAIBOX_DIST_AttachClass))
#define GST_IS_AIBOX_DIST_ATTACH(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_AIBOX_DIST_ATTACH))
#define GST_IS_AIBOX_DIST_ATTACH_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_AIBOX_DIST_ATTACH))

typedef struct _GstAIBOX_DIST_Attach GstAIBOX_DIST_Attach;
typedef struct _GstAIBOX_DIST_AttachClass GstAIBOX_DIST_AttachClass;

struct _GstAIBOX_DIST_Attach
{
  GstBaseTransform parent;
  guint max_num;
  guint camId;
  zmq::context_t zctx;
  zmq::socket_t zsck;
  gboolean bind;
};

struct _GstAIBOX_DIST_AttachClass
{
  GstBaseTransformClass parentclass;
};

GType gst_aibox_dist_attach_get_type (void);

G_END_DECLS

#endif
