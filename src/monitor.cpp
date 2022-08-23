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

#include <stdio.h>
#include <gst/gst.h>
#include <string>
#include <array>
#include <vector>
#include <set>
#include <sstream>
#include <memory>
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <cstdint>
#include <regex>
#include <glib-unix.h>
#include "../cros_mt_reid/src/mtmc_reid.hpp"

static char *msgFirmware = (char *)"Please make sure that the HW accelerator firmware is loaded via xmutil loadapp kv260-aibox-reid.\n";
static gchar* DEFAULT_SRC_TYPE = (gchar*)"r";
static gchar* DEFAULT_SRC_ENC = (gchar*)"h264";
static gchar* DEFAULT_FRAME_RATE = (gchar*)"auto";
static gchar* DEFAULT_CONFIGPATH = (gchar*)"/opt/xilinx/kv260-aibox-dist/share/vvas/aibox-dist/cam_setup.json";
static gchar** srctypes = NULL;
static gchar** srcencs = NULL;
static gchar** srcs= NULL;
static gchar** poses = NULL;
static gchar** frs = NULL;
static gint w = 1920;
static gint h = 1080;
static gboolean reportFps = FALSE;
static gint syncType = 0;
static gint indTol = 0;
static gint timeTol = 100;
static gint pushPop = 0;
static gchar* configPath = DEFAULT_CONFIGPATH;

static GOptionEntry entries[] =
{
    { "src", 's', 0, G_OPTION_ARG_STRING_ARRAY, &srcs, "URI of rtsp src, or location of h264|h265 video file. Must set. Can set up to 4 times", "[rtsp://server:port/id |file path]"},
    { "syncType", 'S', 0, G_OPTION_ARG_INT, &syncType, "Sync Type: 0: Sync for file based stream; 1: Sync for live stream"},
    { "indTol", 'I', 0, G_OPTION_ARG_INT, &indTol, "Tolerance value used by syncType 0: number of frame"},
    { "timeTol", 'T', 0, G_OPTION_ARG_INT, &timeTol, "Tolerance value used by syncType 1: microsecond"},
    { "srcenc", 'e', 0, G_OPTION_ARG_STRING_ARRAY, &srcencs, "Encoding type of the input source. Optional. Can set up to 4 times.", "[h264|h265]"},
    { "pos", 'p', 0, G_OPTION_ARG_STRING_ARRAY, &poses, "Location of the display in the 4 grids of 4k monitor. Optional. 0: top left, 1: top right, 2: bottom left, 3: bottom right. Optional. Can set up to 4 times.", "[0|1|2|3]"},
    { "framerate", 'r', 0, G_OPTION_ARG_STRING_ARRAY, &frs, "Framerate of the input. Optional. Can set up to 4 times.", DEFAULT_FRAME_RATE},
    { "report", 'R', 0, G_OPTION_ARG_NONE, &reportFps, "Report fps", NULL },
    { "config", 'c', 0, G_OPTION_ARG_STRING, &configPath, "System calibration json file path", DEFAULT_CONFIGPATH},
    { NULL }
};

static gboolean
my_bus_callback (GstBus * bus, GstMessage * message, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_INFO:{
      GError *err;
      gchar *debug;
      gst_message_parse_info (message, &err, &debug);
      g_print ("Info: %s\n", debug);
      g_free(debug);
      g_error_free(err);
      break;
    }
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:{
      GError *err;
      gchar *debug;
      gst_message_parse_error (message, &err, &debug);
      g_print ("Error: %s\n", debug);
      g_free(debug);
      g_error_free(err);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }

  return TRUE;
}

static int GetArgArraySizeCheckSameForNonZero(char** args, int &num, int &numNonZero)
{
    num = 0;
    if (args)
    {
        for (; args[num]; num++) ;
    }
    if (num != 0)
    {
        if (numNonZero != 0 && numNonZero != num)
        {
            return 1;
        }
        numNonZero = num;
    }
    return 0;
}

static std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

static std::string GetHostFromRtspAddress(std::string addr)
{
    std::regex rgx(".*rtsp://(.*):.*");
    std::smatch match;
    if ( std::regex_search(addr, match, rgx) )
        return match[1];
    else
        return "";
}

static gboolean
intr_handler (gpointer user_data)
{
  printf("handling interrupt.\n");
  GMainLoop *loop = (GMainLoop *) user_data;

  g_main_loop_quit (loop);
  return G_SOURCE_REMOVE;
}

int
main (int argc, char *argv[])
{
    char* pathVar = std::getenv("PATH");
    std::string setPath = std::string("PATH=") + std::string(pathVar) + ":/usr/sbin:/sbin";
    putenv((char*)setPath.c_str());

    pathVar = std::getenv("LD_LIBRARY_PATH");
    setPath = std::string("LD_LIBRARY_PATH=/opt/xilinx/kv260-aibox-dist/lib:") + (pathVar ? std::string(pathVar) : "");
    putenv((char*)setPath.c_str());

    pathVar = std::getenv("GST_PLUGIN_PATH");
    setPath = std::string("GST_PLUGIN_PATH=/opt/xilinx/kv260-aibox-dist/lib:") + (pathVar ? std::string(pathVar) : "");
    putenv((char*)setPath.c_str());

    GMainLoop *loop;
    GOptionContext *optctx;
    GError *error = NULL;
    guint busWatchId;

    optctx = g_option_context_new ("- AI Application of pedestrian + reid + tracking for multi RTSP streams, on SoM board of Xilinx.");
    g_option_context_add_main_entries (optctx, entries, NULL);
    g_option_context_add_group (optctx, gst_init_get_option_group ());
    if (!g_option_context_parse (optctx, &argc, &argv, &error)) {
        g_printerr ("Error parsing options: %s\n", error->message);
        g_option_context_free (optctx);
        g_clear_error (&error);
        return -1;
    }
    g_option_context_free (optctx);
    g_clear_error (&error);

    if (getuid() != 0) {
      g_printerr ("Please run with sudo.\n");
      return 1;
    }

    std::string confdir("/opt/xilinx/kv260-aibox-dist/share/vvas/");
    char pip[12500];
    pip[0] = '\0';
    sprintf(pip + strlen(pip), "aibox_dist_sum name=sum sync-type=%d tol=%d tol-time=%d push-all-pop=%d delay=10000 timeout=-1 use-pts=1 config=%s "
            , syncType ? 1 : 0, indTol, timeTol, pushPop, configPath);

    char *perf = (char*)"";
    if (reportFps)
    {
        perf = (char*)"! perf ";
    }

    if (access("/dev/allegroDecodeIP", F_OK) != 0)
    {
        g_printerr("ERROR: VCU decoder is not ready.\n%s", msgFirmware);
        return 1;
    }

    if (access("/dev/dri/by-path/platform-b0000000.v_mix-card", F_OK) != 0)
    {
        g_printerr("ERROR: Mixer device is not ready.\n%s", msgFirmware);
        return 1;
    }
    else 
    {
        exec("echo | modetest -M xlnx -D b0000000.v_mix -s 52@40:3840x2160@NV16");
    }

    int numNonZero = 0, numSrcs = 0, numSrcTypes = 0, numSrcEncs = 0, numPoses = 0, numFrs = 0;
    if ( GetArgArraySizeCheckSameForNonZero(srcs,     numSrcs,     numNonZero) != 0
      || GetArgArraySizeCheckSameForNonZero(srcencs,  numSrcEncs,  numNonZero) != 0
      || GetArgArraySizeCheckSameForNonZero(frs,       numFrs,       numNonZero) != 0
      || GetArgArraySizeCheckSameForNonZero(poses,    numPoses,    numNonZero) != 0 ) 
    {
        g_printerr("Error: The num of args srcenc, framerate, and pos should be the same as num of src, if you need to set them to non-empty.\n");
        return 1;
    }

    if (numNonZero > 4) 
    {
        g_print("Warning: Srcs are more than 4, and only the first 4 will be used.");
    }

    std::set<int> posSet;
    int validsrc = 0;
    for (int i = 0; i < numSrcs; i++)
    {
        int pos;
        if (numPoses == numSrcs)
        {
            char* tmp = poses[i];
            if (std::string(tmp) == "n")
            {
                pos = -1;
            }
            else
            {
                pos = atoi(tmp);
                if ( pos < 0 || pos > 3 ) {
                    g_printerr("Error: Src %d 's position %s is invalid, should be in [0, 3]. This src will not display.\n", i, tmp);
                    continue;
                }
                if ( ! posSet.insert(pos).second ) {
                    g_printerr("Error: Src %d 's position %s is duplicated with others, please choose different position. This src will not display.\n", i, tmp);
                    continue;
                }
            }
        }
        else
        {
            pos = validsrc;
        }
        char* srcenc = numSrcEncs == numSrcs ? srcencs[i] : DEFAULT_SRC_ENC;
        char* fr = numFrs == numSrcs ? frs[i] : DEFAULT_FRAME_RATE;

        std::ostringstream framerateOss;
        long ret=0;
        char* tmp = NULL;
        ret = strtol(fr, &tmp, 10);
        if (strlen(tmp) != 0 || ret == 0)
        {
            framerateOss << "";
        }
        else
        {
            framerateOss << ", framerate=" << ret << "/1";
        }

        std::string src(srcs[i]);
        std::string host = GetHostFromRtspAddress(src);
        if (host == "")
        {
            g_printerr("Error: Error occurs when extract hostname from No. %d RTSP address \"%s\".\n", i, src.c_str());
            return 1;
        }
        std::ostringstream srcOss;
        std::string queue_beforedec = "";
        std::string queue_parse = "";
        std::string queue = "";

        {
            queue_beforedec = " queue  leaky=2 max-size-buffers=0 max-size-bytes=0 max-size-time=30000000000  ";
            queue_parse     = " queue  leaky=0 max-size-buffers=0 max-size-bytes=0 max-size-time=0  ";
            srcOss << "rtspsrc location=\"" << src 
                   << "\" " << " latency=40000 " << " ! " << queue_beforedec << " name=depay_" << i << " ! rtpmp2tdepay ! " << queue_beforedec << " ! tsparse ! video/mpegts !  " << queue_beforedec << "  ! tsdemux ";
            queue = " ! queue ";
        }

        sprintf(pip + strlen(pip),
                " %s \
                ! %sparse ! %s ! omx%sdec name=\"dec-%d\" qos=false \
                ! video/x-raw, format=NV12 %s ! %s \
                ! aibox_dist_parse max-num=%d host=%s use-pts=1 result-limit=40000 \
                ! queue leaky=%d max-size-buffers=%d max-size-bytes=0 max-size-time=0 \
                ! sum.sink_%d sum.src_%d ! %s \
                ! vvas_xfilter kernels-config=\"%s/draw_reid.json\" \
                %s "
                , srcOss.str().c_str()
                , srcenc, queue_beforedec.c_str(), srcenc, i
                , framerateOss.str().c_str(), queue_parse.c_str()
                , i, host.c_str() 
                , syncType == 0 ? 0 : 0, 0
                , i, i, queue_parse.c_str()
                , confdir.c_str()
                , perf
               );

        if (pos >= 0 && pos <= 3)
        {
            sprintf(pip + strlen(pip),
                "! kmssink bus-id=b0000000.v_mix plane-id=%d \
                render-rectangle=\"<%d,%d,1920,1080>\" show-preroll-frame=false \
                sync=%s "
                , 34+validsrc
                , pos%2*1920, pos/2*1080
                , std::string(srcenc)=="h265" && std::string(fr)!="30" ? "true" : "false"
                );
        }
        else
        {
            sprintf(pip + strlen(pip),
                    "! fakesink ");
        }

        validsrc++;
    }


    if (validsrc > 0)
    {
        loop = g_main_loop_new (NULL, FALSE);
        GstElement *pipeline = gst_parse_launch(pip, NULL);
        gst_element_set_state (pipeline, GST_STATE_PLAYING);
        /* Wait until error or EOS */
        GstBus *bus = gst_element_get_bus (pipeline);
        busWatchId = gst_bus_add_watch (bus, my_bus_callback, loop);

        int signal_watch_intr_id = g_unix_signal_add (SIGINT,
                       (GSourceFunc) intr_handler, loop);

        g_main_loop_run (loop);

        gst_object_unref (bus);
        gst_element_set_state (pipeline, GST_STATE_NULL);
        gst_object_unref (pipeline);
        g_source_remove (busWatchId);
        g_main_loop_unref (loop);
    }
    else
    {
        printf("Error: No valid input source.\n");
    }
    return 0;
}
