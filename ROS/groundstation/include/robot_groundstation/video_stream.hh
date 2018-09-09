#ifndef _ROBOLAND_VIDEO_STREAM_ROS_HH_
#define _ROBOLAND_VIDEO_STREAM_ROS_HH_

#include "robot_groundstation/video_stream.hh"

// Standard C++ Libraries
#include <iostream>
#include <sstream>
#include <string>

// Boost Libraries
#include <boost/asio.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// GStreamer
#include "glib.h"
#include <gst/gst.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <gst/gstutils.h>
#include <gst/app/gstappsrc.h>

// OpenCV
// #include "cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

namespace roboland 
{

class VideoStream 
{
public:
   VideoStream(int argc, char *argv[]);
  ~VideoStream();

   boost::shared_ptr<boost::asio::io_service> io_service;
   boost::shared_ptr<boost::asio::io_service::work> work;
   boost::shared_ptr<boost::thread_group> threadgroup;

   GstElement *pipeline;
   GstElement *converter_FFMpegColorSpace;
   GstElement *converter_VP8_Encoder;
   GstElement *muxer_WebM;
   GstElement *sink_TCPServer;

   GMainLoop *glib_MainLoop;
   unsigned int heartbeat_Intervall; ///< In Milliseconds
   boost::shared_ptr<boost::asio::deadline_timer> heartbeat;

   GstElement *source_OpenCV;
   guint64 imagecounter;

   void GLib_MainLoop();
   void Create_PipelineGraph( GstElement *pipeline );
   void Push_new_Image();
   cv::Mat Create_Image();

   boost::thread t;
   void thread();
   bool started;
   bool ready;
};

} // namespace roboland

#endif /* _ROBOLAND_VIDEO_STREAM_ROS_HH_ */
