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

#include <pcl/search/kdtree.h>

namespace po = boost::program_options;

typedef pcl::PointXYZRGB PointT;

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
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());

        pcl::DoNFilter<PointT> donf;
        donf.setInputCloud (cloud);
        donf.setSearchMethod (tree);
        donf.setScaleSmall (scale1);
        donf.setScaleSmall (scale2);
        donf.filter (outcloud);

        // Save filtered output
        pcl::io::savePCDFile (outfile.c_str (), outcloud);
        return (0);
}
