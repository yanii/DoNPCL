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
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>

using namespace pcl;
using namespace std;

namespace po = boost::program_options;

int main(int argc, char *argv[])
{
	///The input
	string infile;
        ///The output
        string outfile;

	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		//Program mode option
		("help", "produce help message")
		//Options
		("infile", po::value<string>(&infile)->required(), "file to read from")
		("outfile", po::value<string>(&outfile)->required(), "the file to write to")
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

	cout << infile << " -> " << outfile << endl;

        pcl::PCDWriter writer;
	writer.writeBinaryCompressed (outfile, blob);

}
