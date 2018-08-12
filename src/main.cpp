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

#include <boost/make_shared.hpp> // allows for copying of pointers


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
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> data_s;
		std::string infile;
		
		// Need to push back each .pcd file into the data_s vector
		for(int i = 0; i < 24; i++)
		{
			infile = filedir;
			infile = infile.append(std::to_string(i+1));
			infile = infile.append(".pcd");

			std::cout << " Loading File: " << infile << std::endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr data (new pcl::PointCloud<pcl::PointXYZRGB>);
			if (pcl::io::loadPCDFile (infile, *data) < 0)  
			{
				std::cout << "Error loading point cloud " << infile << std::endl << std::endl;
				return -1;
			}

			data_s.push_back(data); // pushing the .pcd data into the data_s vector

		}

		int v1, v2; // initializing the viewports
	
		if (data_s.empty())
		{
			PCL_ERROR ("The data_s vector is empty! \n") ;
			return (-1);
		}

		pcl::visualization::PCLVisualizer *p;
		p = new pcl::visualization::PCLVisualizer("Point Cloud Window");
	//	p->createViewPort(0.0, 0, 0.5, 1.0, v1); // the sizes of each viewport
	//	p->createViewPort(0.5, 0, 1.0, 1.0, v2);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>), source, target;
		//std::cout << data_s.size() << std::endl;
		for(size_t i = 1; i < data_s.size(); ++i)
		{
			source = data_s[i-1];
			target = data_s[i];

			p->removePointCloud ("_target");
  			p->removePointCloud ("_source");
			
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h (target);
  			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_h (source);
			
			


			//Rotation Function ------------------------------------------------------------------------------------------------------------------------------------
			float angle = 3.14159;   // angle of rotation, radians = 180 degrees
			// Using Eigen::Affine3f for 3D point cloud
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			Eigen::Affine3f shift_right = Eigen::Affine3f::Identity();
			shift_right.translation() << 0.25, 0.0,1.0; // translate with zoom and shift
			transform.translation() << 0.0, 0.0, 1.0; // translate with a zoom in Z-axis
			transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX())); // x rotation
			shift_right.rotate(Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX())); // same rotation as before
			//std::cout << transform.matrix() << std::endl; // display the rotation matrix
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source (new pcl::PointCloud<pcl::PointXYZRGB> ()); // doing the transformation
		 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_target (new pcl::PointCloud<pcl::PointXYZRGB> ());
			 // applying the transformation to the cloud
			pcl::transformPointCloud (*source, *transformed_source, transform);
			// Translation for second point cloud to keep in the same viewport, rather than two
			pcl::transformPointCloud (*target, *transformed_target, shift_right);

			p->addPointCloud (transformed_target, tgt_h, "_target");
  			p->addPointCloud (transformed_source, src_h, "_source");  

			// Save the source cloud to the output directory 
			std::string outfile = filedir;
			outfile.append("output/");
			outfile.append(std::to_string(i));
			outfile.append(".pcd");
			std::cout << "trying to save Point cloud to: " << outfile << std::endl;
			pcl::io::savePCDFileASCII(outfile, *transformed_source);
			std::cout << "Point cloud saved to: " << outfile << std::endl;

 			p->spin();
		}

	return 0;
} //END of MAIN
