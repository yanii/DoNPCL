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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/don.h>

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
class OpenNIDoNEstimation
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename pcl::search::Search<PointType>::Ptr SearchPtr;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIDoNEstimation (float scale_s, float scale_l, const std::string& device_id = "")
      : viewer ("PCL OpenNI DoN Viewer")
    , device_id_(device_id), scale_s_(scale_s), scale_l_(scale_l)
    {
      //ne_.setNormalSmoothingSize (11.0);
      new_cloud_ = false;
      viewer.registerKeyboardCallback(&OpenNIDoNEstimation::keyboard_callback, *this);
    }


    void
    cloud_cb (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (mtx_);
      //lock while we set our cloud;
      FPS_CALC ("computation");
      // Estimate surface normals

      normals_small_scale_.reset (new pcl::PointCloud<pcl::PointNormal>);
      normals_large_scale_.reset (new pcl::PointCloud<pcl::PointNormal>);
      don_.reset (new pcl::PointCloud<pcl::PointNormal>);

      double start = pcl::getTime ();

      ne_.setInputCloud (cloud);

      /**
       * NOTE: setting viewpoint is very important, so that we can ensure
       * normals are all pointed in the same direction!
       */
      ne_.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());

      if(scale_l_ >= scale_s_){
        cerr << "Error: Large scale must be > small scale!" << endl;
        exit(EXIT_FAILURE);
      }

      if (cloud->isOrganized ())
      {
        tree_.reset (new pcl::search::OrganizedNeighbor<PointType> ());
      }
      else
      {
        /**
         * NOTE: Some PCL versions contain a KDTree with a critical bug in
         * which setSearchRadius is ineffective (always uses K neighbours).
         *
         * Since DoN *requires* a fixed search radius, if you are getting
         * strange results in unorganized data, compare them with that
         * while using the Octree search method.
         */
        //tree.reset (new pcl::search::Octree<PointType> (scale1/10));
        tree_.reset (new pcl::search::KdTree<PointType> (false));
      }
      tree_->setInputCloud (cloud);

      ne_.setSearchMethod (tree_);

      //the normals calculated with the small scale
      cout << "Calculating normals for scale..." << scale_s_ << endl;
      ne_.setRadiusSearch (scale_s_);
      ne_.compute (*normals_small_scale_);

      //the normals calculated with the small scale
      cout << "Calculating normals for scale..." << scale_l_ << endl;
      ne_.setRadiusSearch (scale_l_);
      ne_.compute (*normals_large_scale_);

      cout << "Calculating DoN... " << endl;

      // Create DoN operator
      done_.setInputCloud (cloud);
      done_.setNormalScaleLarge(normals_large_scale_);
      done_.setNormalScaleSmall(normals_small_scale_);

      if(!done_.initCompute ()){
        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
      }

      pcl::copyPointCloud<PointType, pcl::PointNormal>(*cloud, *don_);

      //Compute DoN
      done_.computeFeature(*don_);

      writer.write<PointType> ("kinect.pcd", *cloud, false);
      writer.write<pcl::PointNormal> ("kinectdon.pcd", *don_, false);

      float threshold = 0.25;

      cout << "Filtering out DoN mag <= "<< threshold <<  "..." << endl;

      // build the condition
      pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (new
            pcl::ConditionOr<pcl::PointNormal> ());
      range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
            pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));
      // build the filter
      pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
      condrem.setInputCloud (don_);

      pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

      // apply filter
      condrem.filter (*doncloud_filtered);

      don_ = doncloud_filtered;

      // Save filtered output
      std::cout << "Filtered Pointcloud: " << don_->points.size () << " data points." << std::endl;
      writer.write<pcl::PointNormal> ("kinectdonfilt.pcd", *don_, false);

      double stop = pcl::getTime ();
      std::cout << "Time for DoN estimation: " << (stop - start) * 1000.0 << " ms" << std::endl;
      cloud_ = cloud;

      new_cloud_ = true;
    }

    void
    viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
      mtx_.lock ();
      if (!cloud_ || !don_)
      {
        //boost::this_thread::sleep(boost::posix_time::seconds(1));
        mtx_.unlock ();
        return;
      }

      CloudConstPtr temp_cloud;
      pcl::PointCloud<pcl::PointNormal>::Ptr temp_normals;
      temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
      temp_normals.swap (don_);
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
        //viz.addPointCloudNormals<PointType, pcl::PointNormal> (temp_cloud, temp_normals, 100, 0.05f, "normalcloud");
        viz.addPointCloud<pcl::PointNormal> (temp_normals, "normalcloud");

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
          //ne_.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointType, pcl::PointNormal>::COVARIANCE_MATRIX);
          //std::cout << "switched to COVARIANCE_MATRIX method\n";
          break;
      }
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id_);

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&OpenNIDoNEstimation::cloud_cb, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);

      viewer.runOnVisualizationThread (boost::bind(&OpenNIDoNEstimation::viz_cb, this, _1), "viz_cb");

      interface->start ();

      while (!viewer.wasStopped ())
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
      }

      interface->stop ();
    }

    pcl::NormalEstimationOMP<PointType, pcl::PointNormal> ne_;
    pcl::DifferenceOfNormalsEstimation<PointType, pcl::PointNormal, pcl::PointNormal> done_;
    pcl::visualization::CloudViewer viewer;
    pcl::PCDWriter writer;
    std::string device_id_;
    boost::mutex mtx_;
    // Data
    float scale_s_;
    float scale_l_;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale_;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale_;
    pcl::PointCloud<pcl::PointNormal>::Ptr don_;
    SearchPtr tree_;
    CloudConstPtr cloud_;
    bool new_cloud_;
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

  std::cout << "<Q,q> quit\n\n";

  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    PCL_INFO ("PointXYZRGBA mode enabled.\n");
    OpenNIDoNEstimation<pcl::PointXYZRGBA> v (1, 0.1, "");
    v.run ();
  }
  else
  {
    PCL_INFO ("PointXYZ mode enabled.\n");
    OpenNIDoNEstimation<pcl::PointXYZ> v (1, 0.1, "");
    v.run ();
  }

  return (0);
}
