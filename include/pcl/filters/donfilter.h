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
#ifndef PCL_FILTERS_DON_H_
#define PCL_FILTERS_DON_H_

#include <pcl/filters/filter.h>
#include <pcl/search/search.h>

namespace pcl
{
  /** \brief A Difference of Normals (DoN) scale filter implementation for point cloud data. Uses the RGB color data channels.
   * \note For more information please see
   * <b>Yani Ioannou. Automatic Urban Modelling using Mobile Urban LIDAR Data.
   * Thesis (Master, Computing), Queen's University, March, 2010.</b>
   * \author Yani Ioannou.
   */
  template<typename PointT>
    class DoNFilter : public Filter<PointT>
    {
      using Filter<PointT>::input_;
      typedef typename Filter<PointT>::PointCloud PointCloud;
      typedef typename pcl::search::Search<PointT>::Ptr SearchPtr;
    private:
      ///The smallest radius (scale) used in the DoN filter.
      double scaleSmall;
      ///The largest radius (scale) used in the DoN filter.
      double scaleLarge;
      ///The search tree used for nearest neighbours;
      SearchPtr tree_;
    public:

      /**
       * Apply the DoN filter to the given pointcloud.
       */
      void
      applyFilter (PointCloud &output);

      /**
       * Creates a new Difference of Normals filter based on the given small and large radius.
       */
      DoNFilter () :
          scaleSmall (0), scaleLarge (std::numeric_limits<double>::max ())
      {
        //
      }

      /**
       * Set the smaller radius (scale) of the DoN filter.
       * @param scale_small the smaller radius (scale) of the DoN filter.
       */
      void
      setScaleSmall (const double scale_small)
      {
        scaleSmall = scale_small;
      }

      /**
       * Get the smaller radius (scale) of the DoN filter.
       * @return the smaller radius (scale) of the DoN filter.
       */
      double
      getScaleSmall ()
      {
        return (scaleSmall);
      }

      /**
       * Set the larger radius (scale) of the DoN filter.
       * @param scale_small the larger radius (scale) of the DoN filter.
       */
      void
      setScaleLarge (const double scale_large)
      {
        scaleLarge = scale_large;
      }

      /**
       * Get the larger radius (scale) of the DoN filter.
       * @return the larger radius (scale) of the DoN filter.
       */
      double
      getScaleLarge ()
      {
        return (scaleLarge);
      }

      /**
       * Set the search tree used for nearest neighbours by the DoN filter.
       * @param tree the search tree used for nearest neighbours by the DoN filter.
       */
      void
      setSearchMethod (const SearchPtr &tree)
      {
        tree_ = tree;
      }
    };
}

#endif // PCL_FILTERS_DON_H_
