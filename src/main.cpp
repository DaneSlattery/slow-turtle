#include <iostream>
#include <boost/thread/thread.hpp>
#include <unistd.h> // for delay
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h> // Allows for .pcd to be read from disk
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // allows for transforms
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>


// The USAGE of the program
void printUsage (const char* progName)
{
    	std::cout << "Usage: " << progName << "[options]\n" << "Options:\n" << "-h for help\n" << "[file_name.pcd]" << std::endl; 
}

void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_out)
{
	float mean = 50;
	float stddev = 1;
	
	float startz = 0;
	float endz = 1; //z
	
	float startx = -0.05;
	float endx = 0.05; //x
	
	float starty = -1; 
	float endy = 1; //y
	
	// z pass through filter
	pcl::PassThrough<pcl::PointXYZRGB> passz;
	passz.setInputCloud(pointcloud_in);
	passz.setFilterFieldName("z");
	passz.setFilterLimits(startz, endz);
	passz.filter(*pointcloud_out);

	// x pass through filter
	pcl::PassThrough<pcl::PointXYZRGB> passx;
	passx.setInputCloud(pointcloud_out);
	passx.setFilterFieldName("x");
	passx.setFilterLimits(startx, endx);
	passx.filter(*pointcloud_out);

	// y pass through filter
	pcl::PassThrough<pcl::PointXYZRGB> passy;
	passy.setInputCloud(pointcloud_out);
	passy.setFilterFieldName("y");
	passy.setFilterLimits(starty, endy);
	passy.filter(*pointcloud_out);


	// statistical filter the point cloud
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (pointcloud_out);
	sor.setMeanK(mean);
	sor.setStddevMulThresh(stddev);
	sor.filter(*pointcloud_out);
}

int main (int argc, char** argv)
{ // START of MAIN

	std::string filedir;
	
	if (argc > 1)
	{
		filedir = argv[1];
		std::string sys_cmd = "mkdir -p ";
		sys_cmd.append(filedir);
		sys_cmd.append("output/");
		system(sys_cmd.c_str());
		std::cout << sys_cmd << std::endl;
	}
	else
	{
		std::cout << "Input path not specified" << std::endl;
		std::cout << "\t Usage: ./exec ./data/dataset/" << std::endl;
		return -1;
	}

	// This is where we want to read in our own .pcd files to view on the viewer
	// ------------------------------------------------------------------------------------------------------

	//pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	// std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd"); // fetches files from args


	//Creating the viewer
	pcl::visualization::PCLVisualizer viewer("Point Cloud Visualizer");
	//setting the background to black
	viewer.setBackgroundColor (0, 0, 0); // set background to black

	// if (!pcd_filename_indices.empty ()) // checks to make sure a .pcd has been specified
	// {
		// Need to call updatePointCloud() to change the point cloud each iteration
		// Creating for loop to go through each .pcd in the file specified
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ()); // create the PCL to read
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ()); // create the PCL to read
		viewer.spinOnce();
		std::string infile;
		for(int i = 0; i < 24; i++)
		{
			infile = filedir;
			infile = infile.append(std::to_string(i+1));
			infile = infile.append(".pcd");

			// Load the PCD files from disk
			// if the file could not be opened then ...
			std::cout << "On file: " << infile << std::endl;
			if (pcl::io::loadPCDFile (infile, *point_cloud_ptr) < 0)  
			{
				std::cout << "Error loading point cloud " << infile << std::endl << std::endl;
				return -1;
			}
			
			filterPointCloud(point_cloud_ptr, filtered_point_cloud);
			
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(filtered_point_cloud); // define the RGB colours
			// Want to rotate the point cloud as it is upside down

			float angle = 3.14159;   // angle of rotation, radians = 180 degrees
			// Using Eigen::Affine3f for 3D point cloud
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.translation() << 0.0, 0.0, 0.0; // translate
			// transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitZ())); // rotation
			transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX())); // x rotation
			//std::cout << transform.matrix() << std::endl; // display the rotation matrix
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ()); // doing the transformation

			// applying the transformation to the cloud
			pcl::transformPointCloud (*filtered_point_cloud, *transformed_cloud, transform);

			if (i != 0)
			{
				viewer.removePointCloud("point_cloud"+(i-1));
			}

			viewer.addPointCloud(transformed_cloud,rgb,"point_cloud"+i);
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud"+i);

			std::string outfile = filedir;
			outfile.append("output/");
			outfile.append(std::to_string(i+1));
			outfile.append(".pcd");
			std::cout << "trying Point cloud saved to: " << outfile << std::endl;

			pcl::io::savePCDFileASCII(outfile, *transformed_cloud);
			std::cout << "Point cloud saved to: " << outfile << std::endl;
			viewer.spin();
		}
	// }
	return 0;
} //END of MAIN
