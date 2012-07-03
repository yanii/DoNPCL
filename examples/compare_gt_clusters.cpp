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

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointNormal PointOutT;
typedef pcl::search::Search<PointT>::Ptr SearchPtr;

const double EPSILON = 0.0001;

int main(int argc, char *argv[])
{
	///The input ground truth.
	string infile;

        ///The original cloud.
        string originalcloud;

	///The candidate point clouds
	vector<string> candidates;

	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Program mode option
		("help", "produce help message")
		//Options
		("groundtruth", po::value<string>(&infile)->required(), "the file to read a ground truth point cloud from")
		("originalcloud", po::value<string>(&originalcloud), "the file to read the original point cloud from (to calculate precision/recall)")
		("candidates", po::value<vector<string> >(&candidates)->required(), "the file(s) to read candidate point cloud from")
		;

        po::positional_options_description p;
        p.add("candidates", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                  options(desc).positional(p).run(), vm);

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
        pcl::fromROSMsg (blob, *gt);

        pcl::PointCloud<PointT>::Ptr original (new pcl::PointCloud<PointT>);

        // Load cloud in blob format
        pcl::io::loadPCDFile (originalcloud.c_str(), blob);
        pcl::fromROSMsg (blob, *original);

        cout << "#"<< "ground truth" << ", " << "ground truth size" << ", " << "candidate" << ", " << "candidate size" << ", " << "original pointcloud" << ", "  << "original pointcloud size" << ", " << "setintersection" << ", " << "setunion" << ", "<< "intersection/union" << ", "
            << "true positives" << ", "
            //<< "true negatives" << ", "
            << "false positives" << ", "
            //<< "falsenegatives" << ", "
            << "precision" << ", " << "recall" //<< ", "
            //<< "specificity"
            << endl;

	SearchPtr tree;

	if (gt->isOrganized ())
	{
	  tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
	}
	else
	{
          tree.reset (new pcl::search::KdTree<PointT> (false));
	}
        tree->setInputCloud (gt);

        pcl::PointCloud<PointT>::Ptr candidate;

        float maxmetric = 0;
        unsigned int maxintersection = 0;
        unsigned int maxunion = 0;
        string maxcfile;
        unsigned int maxcfilesize = 0;

        unsigned int setintersection = 0;
        unsigned int setunion = 0;
        float metric = 0;

        //int truenegatives = 0;
        int truepositives = 0;
        int falsepositives = 0;
        //int falsenegatives = 0;

        float precision = 0;
        float recall = 0;
        //float specificity = 0;

        for(vector<string>::iterator cfile = candidates.begin(); cfile != candidates.end(); cfile++){
          pcl::io::loadPCDFile (cfile->c_str(), blob);
          candidate.reset(new pcl::PointCloud<PointT>);

          pcl::fromROSMsg (blob, *candidate);

          //find the nearest neighbour for each point within numerical error EPSILON
          std::vector< std::vector< int > > k_indices;
          std::vector< std::vector< float > > k_sqr_distances;
          tree->setSortedResults (false);
          tree->radiusSearch  (*candidate, vector<int>(), EPSILON, k_indices, k_sqr_distances, 1);

          std::vector<int> intersection;

          //For the set, calculate set union and set intersection
          for (unsigned int i = 0; i < k_indices.size(); i++)
          {
            if(!k_indices[i].empty())
            {
              intersection.push_back(k_indices[i][0]);
            }
          }

          setintersection = intersection.size();
          setunion = gt->size() + candidate->size() - setintersection;
          metric = (((float)setintersection)/((float)setunion));

          if(setintersection == 0 || maxintersection > setintersection){
            continue;
          }else{
            maxintersection = setintersection;
            maxunion = setunion;
            maxmetric = metric;
            maxcfile = *cfile;
            maxcfilesize = cfile->size();
          }

          truepositives = setintersection;
          falsepositives = candidate->size() - truepositives;

          //find number of falsenegatives
          SearchPtr ctree;

          if (candidate->isOrganized ())
          {
            ctree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
          }
          else
          {
            ctree.reset (new pcl::search::KdTree<PointT> (false));
          }
          ctree->setInputCloud (candidate);
          //find the nearest neighbour for each point within numerical error EPSILON
          std::vector< std::vector< int > > gt_indices;
          std::vector< std::vector< float > > gt_sqr_distances;
          ctree->setSortedResults (false);
          ctree->radiusSearch  (*gt, vector<int>(), EPSILON, gt_indices, gt_sqr_distances, 1);

          //falsenegatives = 0;

          //For the set, calculate set union and set intersection
          /*for (unsigned int i = 0; i < gt_indices.size(); i++)
          {
            if(gt_indices[i].empty())
            {
              falsenegatives++;
            }
          }*/

          //truenegatives = original->size() - truepositives - falsenegatives;

          precision = ((float)truepositives)/(truepositives+falsepositives);
          recall = ((float)truepositives)/((float)gt->size());
          //recall = ((float)truepositives)/(truepositives+falsenegatives);
          //specificity = ((float)truenegatives)/(truenegatives+falsepositives);
        }

        cout << infile << ", " << gt->size() << ", " << maxcfile << ", " << maxcfilesize << ", " << originalcloud << ", "  << original->size() << ", " << maxintersection << ", " << maxunion << ", "<< maxmetric << ", "
            << truepositives << ", "
            //<< truenegatives << ", "
            << falsepositives << ", "
            //<< falsenegatives << ", "
            << precision << ", " << recall// << ", "
            //<< specificity
            << endl;

	return (0);
}
