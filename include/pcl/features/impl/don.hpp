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

#include <pcl/features/don.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointNT, typename PointOutT> bool
pcl::DifferenceOfNormalsEstimation<PointInT, PointNT, PointOutT>::initCompute ()
{
  if (!Feature<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // Check if input normals are set
  if (!input_normals_)
  {
    PCL_ERROR ("[pcl::%s::initCompute] No input dataset containing normals was given!\n", getClassName ().c_str ());
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }

  // Check if the size of normals is the same as the size of the surface
  if (input_normals_->points.size () != input_->points.size ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] ", getClassName ().c_str ());
    PCL_ERROR ("The number of points in the input dataset differs from ");
    PCL_ERROR ("the number of points in the dataset containing the normals!\n");
    Feature<PointInT, PointOutT>::deinitCompute ();
    return (false);
  }
}

#define PCL_INSTANTIATE_DifferenceOfNormalsEstimation(T,NT,OutT) template class PCL_EXPORTS pcl::DifferenceOfNormalsEstimation<T,NT,OutT>;

#endif // PCL_FILTERS_DON_H_
