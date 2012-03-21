/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include "pcl_don.h"

typedef pcl::visualization::PointCloudColorHandler<pcl::Normal> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

template <typename PointType>
class OpenNIIntegralImageNormalEstimation
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIIntegralImageNormalEstimation (unsigned int smallr, unsigned int larger, const std::string& device_id = "")
      : viewer ("PCL OpenNI NormalEstimation Viewer")
    , device_id_(device_id), smallradius(smallr), largeradius(larger)
    {
      //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_3D_GRADIENT);
      ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
      new_cloud_ = false;

      viewer.registerKeyboardCallback(&OpenNIIntegralImageNormalEstimation::keyboard_callback, *this);
    }


    void
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      //lock while we set our cloud;
      FPS_CALC ("computation");
      // Estimate surface normals

      normals_small_.reset (new pcl::PointCloud<pcl::Normal>);
      normals_large_.reset (new pcl::PointCloud<pcl::Normal>);
      donoutput_.reset (new pcl::PointCloud<pcl::Normal>);

      //double start = pcl::getTime ();
      ne_.setInputCloud (cloud);

      ne_.setRectSize(smallradius, smallradius);
      ne_.setNormalSmoothingSize (smallradius);
      ne_.compute (*normals_small_);

      ne_.setRectSize(largeradius, largeradius);
      ne_.setNormalSmoothingSize (largeradius);
      ne_.compute (*normals_large_);

      don_.setInputCloud (cloud);
      don_.setNormalScaleLarge(normals_large_);
      don_.setNormalScaleSmall(normals_small_);

	  if(!don_.initCompute ()){
	    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
	    exit(EXIT_FAILURE);
	  }

      donoutput_->reserve(cloud->size());

      //Compute DoN
      don_.computeFeature(*donoutput_);

      //double stop = pcl::getTime ();
      //std::cout << "Time for don estimation: " << (stop - start) * 1000.0 << " ms" << std::endl;
      cloud_ = cloud;

      new_cloud_ = true;
    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      mtx_.lock ();
      if (!cloud_ || !normals_small_|| !normals_large_ || !donoutput_ )
      {
        //boost::this_thread::sleep(boost::posix_time::seconds(1));
        mtx_.unlock ();
        return;
      }

      CloudConstPtr temp_cloud;
      pcl::PointCloud<pcl::Normal>::Ptr temp_normals;
      temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
      temp_normals.swap (donoutput_);
      mtx_.unlock ();

      if (!viz.updatePointCloud (temp_cloud, "OpenNICloud"))
      {
        viz.addPointCloud (temp_cloud, "OpenNICloud");
        viz.resetCameraViewpoint ("OpenNICloud");
      }
      // Render the data
      if (new_cloud_)
      {
        viz.removePointCloud ("normalcloud");
        viz.addPointCloudNormals<PointType, pcl::Normal> (temp_cloud, temp_normals, 10, 0.2f, "normalcloud");
        //color_handler_.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::Normal> (temp_normals, "curvature"));
        //viz.removePointCloud ("doncloud");
        //viz.addPointCloud<PointType>(temp_cloud, color_handler_, "doncloud");
        new_cloud_ = false;
      }
    }

    void
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      boost::mutex::scoped_lock lock (mtx_);
      switch (event.getKeyCode ())
      {
        case '1':
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::COVARIANCE_MATRIX);
          std::cout << "switched to COVARIANCE_MATRIX method\n";
          break;
        case '2':
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_3D_GRADIENT);
          std::cout << "switched to AVERAGE_3D_GRADIENT method\n";
          break;
        case '3':
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_DEPTH_CHANGE);
          std::cout << "switched to AVERAGE_DEPTH_CHANGE method\n";
          break;
        case '4':
          ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::SIMPLE_3D_GRADIENT);
          std::cout << "switched to SIMPLE_3D_GRADIENT method\n";
          break;
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIIntegralImageNormalEstimation::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      viewer.runOnVisualizationThread (boost::bind(&OpenNIIntegralImageNormalEstimation::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }

    pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne_;
    pcl::DifferenceOfNormalsEstimation<PointType, pcl::Normal, pcl::Normal> don_;
    pcl::visualization::CloudViewer viewer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    pcl::PointCloud<pcl::Normal>::Ptr normals_small_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_large_;
    pcl::PointCloud<pcl::Normal>::Ptr donoutput_;
    CloudConstPtr cloud_;
    bool new_cloud_;
    unsigned int smallradius;
    unsigned int largeradius;
    ColorHandlerPtr color_handler_;
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " [<device_id>]\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (arg == "--help" || arg == "-h" || driver.getNumberDevices () == 0)
  {
    usage (argv);
    return 1;
  }

  unsigned int smallr = 3;
  unsigned int larger = 30;

  std::cout << "Press following keys to switch to the different integral image normal estimation methods:\n";
  std::cout << "<1> COVARIANCE_MATRIX method\n";
  std::cout << "<2> AVERAGE_3D_GRADIENT method\n";
  std::cout << "<3> AVERAGE_DEPTH_CHANGE method\n";
  std::cout << "<4> SIMPLE_3D_GRADIENT method\n";
  std::cout << "<Q,q> quit\n\n";

  pcl::OpenNIGrabber grabber ("");
  /*if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    OpenNIIntegralImageNormalEstimation<pcl::PointXYZRGBA> v (smallr, larger, "");
    v.run ();
  }
  else
  {*/
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIIntegralImageNormalEstimation<pcl::PointXYZ> v (smallr, larger, "");
    v.run ();
  //}

  return (0);
}
