#include <iostream>
#include <boost/thread/thread.hpp>
#include <unistd.h> // for delay
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h> // Allows for .pcd to be read from disk
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> // allows for transforms
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

using std::string;
using std::cout;
using std::endl;
using pcl::io::loadPCDFile;

void filterPointCloud(PointCloudPtr pointcloud_in, PointCloudPtr pointcloud_out)
{
	float mean = 50;
	float stddev = 1;

	// statistical filter the point cloud
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (pointcloud_in);
	sor.setMeanK(mean);
	sor.setStddevMulThresh(stddev);
	sor.filter(*pointcloud_out);
}

int main (int argc, char** argv)
{ // START of MAIN
 	
	string fileDir;
	
	if (argc > 1)
	{
		fileDir = argv[1];
		string sys_cmd = "mkdir -p ";
		sys_cmd.append(fileDir);
		sys_cmd.append("output/");
		system(sys_cmd.c_str());
		cout << sys_cmd << endl;
	}
	else
	{
		cout << "Input path not specified" << endl;
		cout << "\t Usage: ./exec ./data/dataset/" << endl;
		return -1;
	}

	pcl::visualization::PCLVisualizer viewer("PCView");
    viewer.setBackgroundColor(0,0,0);
    viewer.spinOnce();
	
	string inputFileName;
	PointCloudPtr aPC ( new PointCloud);
	PointCloudPtr bPC ( new PointCloud);
	PointCloudPtr outPC ( new PointCloud);
	PointCloudPtr aPCSampled (new PointCloud);
	PointCloudPtr bPCSampled (new PointCloud);

	// load the first file
	inputFileName = fileDir;
	inputFileName.append("1.pcd");
	loadPCDFile(inputFileName, *aPC);

	// iputFileName.append(".pcd");
	for (auto i = 2; i < 4; i++)
	{
		cout << i << endl;
		inputFileName = fileDir;
		inputFileName.append(std::to_string(i));
		inputFileName.append(".pcd");
		// load next file into b
		loadPCDFile(inputFileName, *bPC);
		
		// we have a and b, now filter
		filterPointCloud(aPC, aPC);
		filterPointCloud(bPC, bPC);

		// visualization
		viewer.addPointCloud(aPC, "pca");
		viewer.addPointCloud(bPC, "pcb");
		viewer.spin();
		viewer.removePointCloud("pca");
    	viewer.removePointCloud("pcb");

		// detect features 
		
		// down sample...
		pcl::VoxelGrid<PointT> vox;
		vox.setLeafSize(0.002f, 0.002f, 0.002f);
		vox.setInputCloud(aPC);
		vox.filter(*aPCSampled);
		vox.setInputCloud(bPC);
		vox.filter(*bPCSampled);

		// visualization
		viewer.addPointCloud(aPCSampled, "pca");
		viewer.addPointCloud(bPCSampled, "pcb");
		viewer.spin();
		viewer.removePointCloud("pca");
    	viewer.removePointCloud("pcb");

		// register, moving b to match a.
		pcl::GeneralizedIterativeClosestPoint6D reg;
		// we want to transform bPC to match aPC
		reg.setInputSource(bPCSampled);
		reg.setInputTarget(aPCSampled);
		reg.setMaximumIterations(200);
		reg.setTransformationEpsilon(1e-8);
		reg.setMaxCorrespondenceDistance(0.1);
		reg.setEuclideanFitnessEpsilon(0.0000001);
		// we never actually use outPC...?
		reg.align(*outPC);

		Eigen::Matrix4f FunMat = Eigen::Matrix4f::Identity();
		FunMat = reg.getFinalTransformation();
		std::cout << "has converged?: " << reg.hasConverged() << ". Score: " << reg.getFitnessScore() << std::endl;
		std::cout << FunMat << std::endl;

		pcl::transformPointCloud(*bPC, *bPC, FunMat);

		viewer.addPointCloud(aPC, "pca");
		viewer.addPointCloud(bPC, "pcb");
		viewer.spin();
		viewer.removePointCloud("pca");
    	viewer.removePointCloud("pcb");

		// save the point clouds to the output
		std::string outFileName = fileDir;
		outFileName.append("output/");
		outFileName.append(std::to_string(i));
		outFileName.append(".pcd");
		cout << "trying to save Point cloud to: " << outFileName << endl;
		pcl::io::savePCDFileASCII(outFileName, *aPC);
		cout << "Point cloud saved to: " << outFileName << endl;

		// now move along the list
		// replace a with b
		pcl::copyPointCloud(*bPC, *aPC);

	}
	// // This is where we want to read in our own .pcd files to view on the viewer
	// // ------------------------------------------------------------------------------------------------------

	// //pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	// // std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd"); // fetches files from args


	// //Creating the viewer
	// pcl::visualization::PCLVisualizer viewer("Point Cloud Visualizer");
	// //setting the background to black
	// viewer.setBackgroundColor (0, 0, 0); // set background to black

	// // if (!pcd_filename_indices.empty ()) // checks to make sure a .pcd has been specified
	// // {
	// 	// Need to call updatePointCloud() to change the point cloud each iteration
	// 	// Creating for loop to go through each .pcd in the file specified

	// 	std::vector<PointCloudPtr> data_s;
	// 	viewer.spinOnce();
	// 	std::string infile;

	// 	for(int i = 0; i < 24; i++)
	// 	{
	// 		infile = filedir;
	// 		infile = infile.append(std::to_string(i+1));
	// 		infile = infile.append(".pcd");

	// 		std::cout << " Loading File: " << infile << std::endl;
	// 		PointCloudPtr data (new PointCloud);
	// 		if (pcl::io::loadPCDFile (infile, *data) < 0)  
	// 		{
	// 			std::cout << "Error loading point cloud " << infile << std::endl << std::endl;
	// 			return -1;
	// 		}
	// 		// filter the data
	// 		filterPointCloud(data, data);

	// 		data_s.push_back(data); // pushing the .pcd data into the data_s vector
	// 	}

	// 	if (data_s.empty())
	// 	{
	// 		PCL_ERROR ("The data_s vector is empty! \n") ;
	// 		return (-1);
	// 	}

	// 	pcl::visualization::PCLVisualizer *p;
	// 	p = new pcl::visualization::PCLVisualizer("Point Cloud Window");
	// //	p->createViewPort(0.0, 0, 0.5, 1.0, v1); // the sizes of each viewport
	// //	p->createViewPort(0.5, 0, 1.0, 1.0, v2);

	// 	PointCloudPtr source (new PointCloud);
	// 	PointCloudPtr target (new PointCloud);
	// 	//std::cout << data_s.size() << std::endl;
	// 	for(size_t i = 1; i < data_s.size(); ++i)
	// 	{
	// 		source = data_s[i-1];
	// 		target = data_s[i];

	// 		PointCloudPtr pc1_sample (new PointCloud);
	// 		PointCloudPtr pc2_sample (new PointCloud);

	// 		pcl::VoxelGrid<PointT> vox;
	// 		vox.setLeafSize(0.001f, 0.001f, 0.001f);
	// 		vox.setInputCloud(source);
	// 		vox.filter(*pc1_sample);
	// 		vox.setInputCloud(target);
	// 		vox.filter(*pc2_sample);

	// 		p->removePointCloud ("_target");
  	// 		p->removePointCloud ("_source");

	// 		// manually rotate by 15 degrees
	// 		Eigen::Affine3f myMat = Eigen::Affine3f::Identity();
	// 		float theta = M_PI/24;
	// 		myMat.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
			
	// 		// we rotate first, since we know the clouds are 15degrees apart
	// 		pcl::transformPointCloud(*pc1_sample, *pc1_sample, myMat);
	// 		pcl::transformPointCloud(*source, *source, myMat);

    // 		PointCloudPtr pcout (new PointCloud);

	// 		 // register	
	// 		pcl::GeneralizedIterativeClosestPoint6D reg;
	// 		// we want to transform pc2 to match pc1
	// 		reg.setInputSource(pc1_sample);
	// 		reg.setInputTarget(pc2_sample);
	// 		reg.setMaximumIterations(100);
	// 		reg.setTransformationEpsilon(1e-7);
	// 		reg.align(*pcout);

	// 		Eigen::Matrix4f FunMat = Eigen::Matrix4f::Identity();
	// 		FunMat = reg.getFinalTransformation();
	// 		std::cout << "has converged?: " << reg.hasConverged() << ". Score: " << reg.getFitnessScore() << std::endl;
	// 		std::cout << FunMat << std::endl;

    // 		pcl::transformPointCloud(*pc1_sample, *pc1_sample, FunMat);

	// 		pcl::visualization::PointCloudColorHandlerRGBField<PointT> tgt_h (pc2_sample);
  	// 		pcl::visualization::PointCloudColorHandlerRGBField<PointT> src_h (pc1_sample);
			
			


	// 		//Rotation Function ------------------------------------------------------------------------------------------------------------------------------------
	// 		float angle = 3.14159;   // angle of rotation, radians = 180 degrees
	// 		// Using Eigen::Affine3f for 3D point cloud
	// 		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	// 		Eigen::Affine3f shift_right = Eigen::Affine3f::Identity();
	// 		shift_right.translation() << 0.0, 0.0, 1.0; // translate with zoom and shift
	// 		transform.translation() << 0.0, 0.0, 1.0; // translate with a zoom in Z-axis
	// 		transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX())); // x rotation
	// 		shift_right.rotate(Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX())); // same rotation as before
	// 		//std::cout << transform.matrix() << std::endl; // display the rotation matrix
	// 		PointCloudPtr transformed_source (new PointCloud); // doing the transformation
	// 	 	PointCloudPtr transformed_target (new PointCloud);
	// 		 // applying the transformation to the cloud
	// 		pcl::transformPointCloud (*pc1_sample, *transformed_source, transform);
	// 		// Translation for second point cloud to keep in the same viewport, rather than two
	// 		pcl::transformPointCloud (*pc2_sample, *transformed_target, shift_right);

	// 		p->addPointCloud (transformed_target, tgt_h, "_target");
  	// 		p->addPointCloud (transformed_source, src_h, "_source");  

	// 		// Save the source cloud to the output directory 
	// 		std::string outfile = filedir;
	// 		outfile.append("output/");
	// 		outfile.append(std::to_string(i));
	// 		outfile.append(".pcd");
	// 		std::cout << "trying to save Point cloud to: " << outfile << std::endl;
	// 		pcl::io::savePCDFileASCII(outfile, *transformed_source);
	// 		std::cout << "Point cloud saved to: " << outfile << std::endl;

 	// 		p->spin();
	// 	}
			
			
		
		
	// // }
	return 0;
} //END of MAIN
