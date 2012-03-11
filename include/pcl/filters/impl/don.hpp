#ifndef PCL_FILTERS_DON_IMPL_H_
#define PCL_FILTERS_DON_IMPL_H_

#include <pcl/filters/don.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>


#define PCL_INSTANTIATE_DoNFilter(T) template class PCL_EXPORTS pcl::DoNFilter<T>;

template <typename PointT> double
pcl::DoNFilter<PointT>::computePointWeight(const int pid,const std::vector<int> &indices, const std::vector<float> &distances)
{
	double BF = 0, W = 0;

	// For each neighbor
	for (size_t n_id = 0; n_id < indices.size (); ++n_id)
	{
		double id = indices[n_id];
		double dist = std::sqrt (distances[n_id]);
		double intensity_dist = abs (input_->points[pid].intensity - input_->points[id].intensity);

		/*double weight = kernel (dist, scaleSmall) * kernel (intensity_dist, scaleLarge);

		BF += weight * input_->points[id].intensity;
		W += weight;*/
	}
	return (BF / W);
}

template <typename PointT> void
pcl::DoNFilter<PointT>::applyFilter (PointCloud &output)
{
	if(scaleSmall == 0)
	{
		PCL_ERROR ("[pcl::DoNFilter::applyFilter] Need a scale_small value given before continuing.\n");
		return;
	}
	if (!tree_)
	{
		if (input_->isOrganized ()){
			tree_.reset(new pcl::search::OrganizedNeighbor<PointT>());
		}
		else{
			tree_.reset(new pcl::search::KdTree<PointT> (false));
		}
	}
	tree_->setInputCloud (input_);

	std::vector<int> k_indices;
	std::vector<float> k_distances;

	output = *input_;

	for (size_t point_id = 0; point_id < input_->points.size (); ++point_id)
	{
		tree_->radiusSearch (point_id, scaleSmall * 2, k_indices, k_distances);

		output.points[point_id].intensity = computePointWeight (point_id, k_indices, k_distances);
	}
}
#endif // PCL_FILTERS_DON_H_
