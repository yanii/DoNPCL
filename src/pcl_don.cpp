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

#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace po = boost::program_options;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointNormal PointOutT;
typedef typename pcl::search::Search<PointT>::Ptr SearchPtr;

int main(int argc, char *argv[])
{
	///The smallest scale to use in the DoN filter.
	double scale1;

	///The smallest scale to use in the DoN filter.
	double scale2;

	///The file to read from.
	string infile;

	///The file to output to.
	string outfile;

	///The minimum DoN magnitude to threshold by
	double threshold;

        ///The euclidian cluster distance to use
        double segradius;

	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Program mode option
		("verbose", "display verbose messages")
		("help", "produce help message")
		//Options
		("smallscale", po::value<double>(&scale1)->required(), "the small scale to use in the DoN filter")
		("largescale", po::value<double>(&scale2)->required(), "the large scale to use in the DoN filter")
		("infile", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		("outfile", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to")
		("magthreshold", po::value<double>(&threshold), "the minimum DoN magnitude to filter by")
		("segment", po::value<double>(&segradius), "the file to write the DoN point cloud & normals to")
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


	// Load cloud in blob format
	sensor_msgs::PointCloud2 blob;
	pcl::io::loadPCDFile (infile.c_str(), blob);

	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        cout << "Converting point cloud...";
        PointCloud<pcl::PointXYZ>::Ptr xyzcloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (blob, *xyzcloud);
        copyPointCloud<pcl::PointXYZ, PointT>(*xyzcloud, *cloud);
        cout << "done." << endl;

	int pnumber = (int)cloud->size ();

	SearchPtr tree;

	if (cloud->isOrganized ())
	{
	  tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
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
	  //tree.reset (new pcl::search::Octree<PointT> (0.5));
          tree.reset (new pcl::search::KdTree<PointT> (false));
	}
	tree->setInputCloud (cloud);

	// Compute normals using both small and large scales at each point
	// TODO: Use IntegralImageNormalEstimation for organized data
	pcl::NormalEstimationOMP<PointT, PointNT> ne;
	ne.setInputCloud (cloud);
	ne.setSearchMethod (tree);

	/**
	 * NOTE: setting viewpoint is very important, so that we can ensure
	 * normals are all pointed in the same direction!
	 */
	ne.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());

	if(scale1 >= scale2){
	  cerr << "Error: Large scale must be > small scale!" << endl;
	  exit(EXIT_FAILURE);
	}

	//the normals calculated with the small scale
	cout << "Calculating normals for scale..." << scale1 << endl;
	pcl::PointCloud<PointNT>::Ptr normals_small_scale (new pcl::PointCloud<PointNT>);
	ne.setRadiusSearch (scale1);
	ne.compute (*normals_small_scale);

	cout << "Calculating normals for scale..." << scale2 << endl;
	//the normals calculated with the large scale
	pcl::PointCloud<PointNT>::Ptr normals_large_scale (new pcl::PointCloud<PointNT>);
	ne.setRadiusSearch (scale2);
	ne.compute (*normals_large_scale);

	// Create output cloud for DoN results
	PointCloud<PointOutT>::Ptr doncloud (new pcl::PointCloud<PointOutT>);
	pcl::fromROSMsg (blob, *xyzcloud);
	copyPointCloud<pcl::PointXYZ, PointOutT>(*xyzcloud, *doncloud);

	cout << "Calculating DoN... " << endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<PointT, PointNT, PointOutT> don;
	don.setInputCloud (cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);

	if(!don.initCompute ()){
	  std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
	  exit(EXIT_FAILURE);
	}

	//Compute DoN
	don.computeFeature(*doncloud);

	//Filter by magnitude
	if(vm.count("magthreshold")){
	  cout << "Filtering out DoN mag <= "<< threshold <<  "..." << endl;

	  // build the condition
	  pcl::ConditionOr<PointOutT>::Ptr range_cond (new
		pcl::ConditionOr<PointOutT> ());
	  range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
		pcl::FieldComparison<PointOutT> ("curvature", pcl::ComparisonOps::GT, threshold)));
	  // build the filter
	  pcl::ConditionalRemoval<PointOutT> condrem (range_cond);
	  condrem.setInputCloud (doncloud);

	  pcl::PointCloud<PointOutT>::Ptr doncloud_filtered (new pcl::PointCloud<PointOutT>);

	  // apply filter
	  condrem.filter (*doncloud_filtered);

	  doncloud = doncloud_filtered;
	}

	//Filter by magnitude
	if(vm.count("segment")){
	  cout << "Clustering using EuclideanClusterExtraction with tolerance <= "<< segradius <<  "..." << endl;

	  pcl::search::KdTree<PointOutT>::Ptr segtree (new pcl::search::KdTree<PointOutT>);
	  segtree->setInputCloud (doncloud);

	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<PointOutT> ec;

	  ec.setClusterTolerance (segradius);
	  ec.setMinClusterSize (100);
	  ec.setMaxClusterSize (100000);
	  ec.setSearchMethod (segtree);
	  ec.setInputCloud (doncloud);
	  ec.extract (cluster_indices);

	  pcl::PCDWriter writer;

	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		pcl::PointCloud<PointOutT>::Ptr cloud_cluster (new pcl::PointCloud<PointOutT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
		  cloud_cluster->points.push_back (doncloud->points[*pit]); //*
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << outfile.substr(0,outfile.length()-4) << "_cluster_" << j << ".pcd";
		writer.write<PointOutT> (ss.str (), *cloud_cluster, false); //*
		j++;
	  }
	}

	// Save filtered output
	sensor_msgs::PointCloud2 outblob;
	pcl::toROSMsg(*doncloud, outblob);
	pcl::io::savePCDFile (outfile.c_str (), outblob);

	return (0);
}
