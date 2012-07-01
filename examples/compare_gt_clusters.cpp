/**
 * @file pcl_don.cpp
 * Difference of normals implementation using PCL.
 *
 * @author Yani Ioannou
 * @date 2012-03-11
 */

#include <boost/program_options.hpp>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/don.h>

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointNormal PointOutT;
typedef pcl::search::Search<PointT>::Ptr SearchPtr;

int main(int argc, char *argv[])
{
	///The input ground truth.
	string infile;

	///The candidate point clouds
	vector<string> candidates;

	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Program mode option
		("help", "produce help message")
		//Options
		("groundtruth", po::value<string>(&infile)->required(), "the file to read a ground truth point cloud from")
		("candidates", po::value<vector<string> >(&candidates)->required(), "the file(s) to read candidate point cloud from")
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

	// Load cloud in blob format
	sensor_msgs::PointCloud2 blob;
	pcl::io::loadPCDFile (infile.c_str(), blob);

	pcl::PointCloud<PointT>::Ptr gt (new pcl::PointCloud<PointT>);
        cout << "Loading ground truth point cloud...";
        pcl::fromROSMsg (blob, *gt);
        cout << "done." << endl;

	SearchPtr tree;

	if (gt->isOrganized ())
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
	  //tree.reset (new pcl::search::Octree<PointT> (scale1/10));
          tree.reset (new pcl::search::KdTree<PointT> (false));
	}
        tree->setInputCloud (gt);

        pcl::PointCloud<PointT>::Ptr candidate;

        for(vector<string>::iterator cfile = candidates.begin(); cfile != candidates.end(); cfile++){
          pcl::io::loadPCDFile (cfile->c_str(), blob);
          candidate.reset(new pcl::PointCloud<PointT>);

          cout << "Loading candidate point cloud " << *cfile << "...";
          pcl::fromROSMsg (blob, *candidate);
          cout << "done." << endl;
        }

	return (0);
}
