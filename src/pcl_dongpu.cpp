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

#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/gpu/features/features.hpp>
#include <pcl/gpu/octree/octree.hpp>

namespace po = boost::program_options;

typedef pcl::PointXYZRGB PointT;
typedef pcl::Normal PointNT;
typedef pcl::PointNormal PointOutT;

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

	// Load cloud in blob format
	sensor_msgs::PointCloud2 blob;
	pcl::io::loadPCDFile (infile.c_str(), blob);

	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        cout << "Converting point cloud...";
        PointCloud<pcl::PointXYZ>::Ptr xyzcloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (blob, *xyzcloud);
        copyPointCloud<pcl::PointXYZ, PointT>(*xyzcloud, *cloud);
        cout << "done." << endl;

	cout << "Uploading point cloud to GPU ..." << endl;
	pcl::gpu::Octree::PointCloud cloud_device;
	cloud_device.upload(xyzcloud->points);

	// Compute normals using both small and large scales at each point
	// TODO: Use IntegralImageNormalEstimation for organized data
	pcl::gpu::NormalEstimation ne;
	ne.setInputCloud (cloud_device);

	/**
	 * NOTE: setting viewpoint is very important, so that we can ensure
	 * normals are all pointed in the same direction!
	 */
	ne.setViewPoint(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());

	if(scale1 >= scale2){
	  cerr << "Error: Large scale must be > small scale!" << endl;
	  exit(EXIT_FAILURE);
	}

	//maximum answers for search radius
	//NOTE: lower this if you are running out of GPU memory
	const int max_answers = 500;
	//buffer for results
	cout << "Creating GPU NormalEstimation output dev..." << endl;
	pcl::gpu::Feature::Normals result_device(cloud_device.size());

	//the normals calculated with the small scale
	cout << "Calculating normals for scale..." << scale1 << endl;
	ne.setRadiusSearch (scale1, max_answers);
	ne.compute (result_device);

	//the normals calculated with the small scale
	cout << "Downloading results from GPU..." << endl;
	std::vector<PointXYZ> normals_small_scale_vec(result_device.size());
	result_device.download(normals_small_scale_vec);

        pcl::PointCloud<PointNT>::Ptr normals_small_scale (new pcl::PointCloud<PointNT>);
        for(std::vector<PointXYZ>::iterator resultpt = normals_small_scale_vec.begin(); resultpt != normals_small_scale_vec.end(); resultpt++){
          normals_small_scale->push_back(Normal(resultpt->x, resultpt->y, resultpt->z));
        }

	cout << "Calculating normals for scale..." << scale2 << endl;
	//the normals calculated with the large scale
	ne.setRadiusSearch (scale2, max_answers);
	ne.compute (result_device);

	cout << "Downloading results from GPU..." << endl;
	std::vector<PointXYZ> normals_large_scale_vec(result_device.size());
	result_device.download(normals_large_scale_vec);

        pcl::PointCloud<PointNT>::Ptr normals_large_scale (new pcl::PointCloud<PointNT>);
        for(std::vector<PointXYZ>::iterator resultpt = normals_large_scale_vec.begin(); resultpt != normals_large_scale_vec.end(); resultpt++){
          normals_large_scale->push_back(Normal(resultpt->x, resultpt->y, resultpt->z));
        }

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
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<PointOutT> (ss.str (), *cloud_cluster, false); //*

            //Mesh the cluster

            // Output has the PointNormal type in order to store the normals calculated by MLS
            pcl::PointCloud<pcl::PointNormal> mls_points;

            // Init object (second point type is for the normals, even if unused)
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

            mls.setComputeNormals (true);

            // Set parameters
            mls.setInputCloud (cloud_cluster);
            mls.setPolynomialFit (true);
            mls.setSearchMethod (tree);
            mls.setSearchRadius (segradius/2);

            // Reconstruct
            mls.process (mls_points);
            ss.str("");
            ss << "cloud_cluster_" << j << ".pcd";
            pcl::io::savePCDFile (ss.str(), mls_points);

            j++;
          }
        }

	// Save filtered output
	sensor_msgs::PointCloud2 outblob;
	pcl::toROSMsg(*doncloud, outblob);
	pcl::io::savePCDFile (outfile.c_str (), outblob);

	return (0);
}
