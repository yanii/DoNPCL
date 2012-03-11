#ifndef PCL_FILTERS_DON_H_
#define PCL_FILTERS_DON_H_

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>

namespace pcl
{
template<typename PointT>
	class DoNFilter : public Filter<PointT>
	{
		using Filter<PointT>::input_;
		typedef typename Filter<PointT>::PointCloud PointCloud;
		typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;
	private:
		///The smallest radius (scale) used in the DoN filter.
		double scaleSmall;
		///The largest radius (scale) used in the DoN filter.
		double scaleLarge;
		///The search tree used for nearest neighbours;
		KdTreePtr tree_;
	public:

		/**
		 * Apply the DoN filter to the given pointcloud.
		 */
		void applyFilter(PointCloud &output);

		/**
		 * Compute the point's weight.
		 */
		double computePointWeight (const int pid, const std::vector<int> &indices, const std::vector<float> &distances);

		/**
		 * Creates a new Difference of Normals filter based on the given small and large radius.
		 */
		DoNFilter () : scaleSmall(0),scaleLarge(std::numeric_limits<double>::max())
		{
			//
		}

		/**
		 * Set the smaller radius (scale) of the DoN filter.
		 * @param scale_small the smaller radius (scale) of the DoN filter.
		 */
		void setScaleSmall(const double scale_small)
		{
			scaleSmall = scale_small;
		}

		/**
		 * Get the smaller radius (scale) of the DoN filter.
		 * @return the smaller radius (scale) of the DoN filter.
		 */
		double getScaleSmall()
		{
			return (scaleSmall);
		}

		/**
		 * Set the larger radius (scale) of the DoN filter.
		 * @param scale_small the larger radius (scale) of the DoN filter.
		 */
		void setScaleLarge(const double scale_large)
		{
			scaleLarge = scale_large;
		}

		/**
		 * Get the larger radius (scale) of the DoN filter.
		 * @return the larger radius (scale) of the DoN filter.
		 */
		double getScaleLarge()
		{
			return (scaleLarge);
		}

		/**
		 * Set the search tree used for nearest neighbours by the DoN filter.
		 * @param tree the search tree used for nearest neighbours by the DoN filter.
		 */
		void setSearchMethod(const KdTreePtr &tree)
		{
			tree_ = tree;
		}
	};
}

#endif // PCL_FILTERS_DON_H_
