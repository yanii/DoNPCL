 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2011, Willow Garage, Inc.
  *  Copyright (c) 2012, Yani Ioannou <yani.ioannou@gmail.com>
  *
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
#ifndef PCL_FILTERS_DON_IMPL_H_
#define PCL_FILTERS_DON_IMPL_H_

#include <limits>

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/donfilter.h>

template<typename PointT>
  void
  pcl::DoNFilter<PointT>::applyFilter (PointCloud &output)
  {
    if (scaleSmall == 0)
    {
      PCL_ERROR ("[pcl::DoNFilter::applyFilter] Need a scale_small value given before continuing.\n");
      return;
    }
    if (!tree_)
    {
      if (input_->isOrganized ())
      {
        tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
      }
      else
      {
        tree_.reset (new pcl::search::KdTree<PointT> (false));
      }
    }
    tree_->setInputCloud (input_);

    //use the input point cloud as the output
    output = *input_;

    // Compute normals using both small and large scales at each point
    // TODO: Use IntegralImageNormalEstimation for organized data
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setInputCloud (this->getInputCloud ());
    ne.setSearchMethod (tree_);
    ne.setViewPoint(0,0,std::numeric_limits<float>::max());

    //the normals calculated with the small scale
    pcl::PointCloud<pcl::Normal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (scaleSmall);
    ne.compute (*normals_small_scale);

    //the normals calculated with the large scale
    pcl::PointCloud<pcl::Normal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (scaleLarge);
    ne.compute (*normals_large_scale);

    //perform subtraction
    for (size_t point_id = 0; point_id < input_->points.size (); ++point_id)
    {
      Eigen::Vector3f don = normals_small_scale->at(point_id).getNormalVector3fMap ()
          - normals_large_scale->at(point_id).getNormalVector3fMap ();
      output.at(point_id).r = (don.x()/2.0f+0.5f)*255;
      output.at(point_id).g = (don.y()/2.0f+0.5f)*255;
      output.at(point_id).b = (don.z()/2.0f+0.5f)*255;
    }
  }

#define PCL_INSTANTIATE_DoNFilter(T) template class PCL_EXPORTS pcl::DoNFilter<T>;

#endif // PCL_FILTERS_DON_H_
