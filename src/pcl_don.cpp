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
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

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
          tree.reset (new pcl::search::KdTree<PointT> (false));
        }
	tree->setInputCloud (cloud);

        // Compute normals using both small and large scales at each point
        // TODO: Use IntegralImageNormalEstimation for organized data
        pcl::NormalEstimationOMP<PointT, PointNT> ne;
        ne.setInputCloud (cloud);
        ne.setSearchMethod (tree);
        // NOTE: this is very important, so that we can ensure normals are all pointed in the same direction!
        ne.setViewPoint(0,0,std::numeric_limits<float>::max());

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

        cout << "Filtering out zero DoN" << endl;

        // build the condition
        pcl::ConditionOr<PointOutT>::Ptr range_cond (new
          pcl::ConditionOr<PointOutT> ());
        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
                  pcl::FieldComparison<PointOutT> ("curvature", pcl::ComparisonOps::GT, 0.0)));
        /*
        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
          pcl::FieldComparison<PointOutT> ("normal_x", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
          pcl::FieldComparison<PointOutT> ("normal_y", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
          pcl::FieldComparison<PointOutT> ("normal_z", pcl::ComparisonOps::GT, 0.0)));

        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
          pcl::FieldComparison<PointOutT> ("normal_x", pcl::ComparisonOps::LT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
          pcl::FieldComparison<PointOutT> ("normal_y", pcl::ComparisonOps::LT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<PointOutT>::ConstPtr (new
          pcl::FieldComparison<PointOutT> ("normal_z", pcl::ComparisonOps::LT, 0.0)));
         */
        // build the filter
        pcl::ConditionalRemoval<PointOutT> condrem (range_cond);
        condrem.setInputCloud (doncloud);


        pcl::PointCloud<PointOutT>::Ptr doncloud_filtered (new pcl::PointCloud<PointOutT>);

        // apply filter
        condrem.filter (*doncloud_filtered);


        sensor_msgs::PointCloud2 outblob;
        pcl::toROSMsg(*doncloud_filtered, outblob);

        // Save filtered output
        pcl::io::savePCDFile (outfile.c_str (), outblob);

        // visualize normals
        //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        //viewer.setBackgroundColor (0.0, 0.0, 0.5);
        //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

        //while (!viewer.wasStopped ())
        //{
        //  viewer.spinOnce ();
        //}

        return (0);
}
