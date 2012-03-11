/**
 * @file pcl_don.cpp
 * Difference of normals implementation using PCL.
 *
 * @author Yani Ioannou
 * @date 2012-03-11
 */

#include "pcl_don.h"
#include <boost/program_options.hpp>

#include <string>

namespace po = boost::program_options;

namespace DoN{
	//
}


typedef pcl::PointXYZI PointT;

int main(int argc, char *argv[])
{
	///The smallest scale to use in the DoN filter.
	float scale1;

	///The smallest scale to use in the DoN filter.
	float scale2;

	///The file to read from.
	string infile;

	///The file to output to.
	string outfile;

	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Program mode option
		("verbose", "display verbose messages")
		("help", "produce help message")
		//Options
		("smallscale", po::value<float>(&scale1)->required(), "the small scale to use in the DoN filter")
		("largescale", po::value<float>(&scale2)->required(), "the large scale to use in the DoN filter")
		("infile", po::value<string>(&infile)->required(), "the file to read the classifier state from")
		("outfile", po::value<string>(&outfile)->required(), "the file to read the classifier state from")
		;
	// Parse the command line
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	// Print help
	if (vm.count("help"))
	{
		cout << desc << "\n";
		return false;
	}

	// Process options.
	po::notify(vm);

	//Verbose mode
	bool verbose = vm.count("verbose");


	// Load cloud
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile (infile.c_str(), *cloud);
	int pnumber = (int)cloud->size ();

	// Output Cloud = Input Cloud
	pcl::PointCloud<PointT> outcloud = *cloud;

	// Set up KDTree
	/*pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
	tree->setInputCloud (cloud);
	 */

	// Neighbors containers
	/*std::vector<int> k_indices;
	std::vector<float> k_distances;
	 */
	// Main Loop
	/*for (int point_id = 0; point_id < pnumber; ++point_id)
	{
		float BF = 0;
		float W = 0;

		tree->radiusSearch(point_id, 2 * scale1, k_indices, k_distances);

		// For each neighbor
		for (size_t n_id = 0; n_id < k_indices.size (); ++n_id)
		{
			float id = k_indices.at (n_id);
			float dist = sqrt (k_distances.at (n_id));
			float intensity_dist = abs (cloud->points[point_id].intensity - cloud->points[id].intensity);

			float w_a = G (dist, scale1);
			float w_b = G (intensity_dist, scale2);
			float weight = w_a * w_b;

			BF += weight * cloud->points[id].intensity;
			W += weight;
		}

		outcloud.points[point_id].intensity = BF / W;
	}*/

	// Save filtered output
	pcl::io::savePCDFile (outfile.c_str(), outcloud);
	return (0);
}
