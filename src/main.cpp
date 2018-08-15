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

// includes for feature detection
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/features/normal_3d.h"
#include "pcl/features/pfh.h"
#include <pcl/registration/transforms.h>
#include <Eigen/Core>

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

void detectKeyPoints(PointCloudPtr &key_points_in, pcl::PointCloud<pcl::PointWithScale>::Ptr &key_points_out)
{
	
	// Using SIFT
	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
	//FLANN-based KDTree
	sift_detect.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
	// Detection Parameters - decreasing the min-scale helps detect more features
	sift_detect.setScales(0.001, 3, 3);
	sift_detect.setMinimumContrast(2.0);

	sift_detect.setInputCloud(key_points_in); // detecting on initial a
	
	// Once detected, store them in the output
	sift_detect.compute(*key_points_out);// computing

}

void computePFHFeatures(PointCloudPtr &points, 
						pcl::PointCloud<pcl::Normal>::Ptr &normals,
                        pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
						pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
						{
							// Create a PFHEstimation object
							pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh_est;
							// Set it to use a FLANN-based KdTree to perform its neighborhood searches
							pfh_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
							// Specify the radius of the PFH feature
							pfh_est.setRadiusSearch (feature_radius);
							
							PointCloudPtr keypoints_xyzrgb (new PointCloud);
							pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb); // convert from PointWithScale to XYZRG
							// Use all of the points for analyzing the local structure of the cloud
							pfh_est.setSearchSurface (points);
							pfh_est.setInputNormals (normals);
							// But only compute features at the keypoints
							pfh_est.setInputCloud (keypoints_xyzrgb);
							// Compute the features
							pfh_est.compute (*descriptors_out);
						}

void computeNormals(PointCloudPtr &points, float normal_radius, pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
	pcl::NormalEstimation<PointT, pcl::Normal> norm_est;
	// Use a FLANN-based KdTree to perform neighborhood searches
	norm_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
	// Specify the size of the local neighborhood to use when computing the surface normals
	norm_est.setRadiusSearch (normal_radius);
	// Set the input points
	norm_est.setInputCloud (points);
	// Estimate the surface normals and store the result in "normals_out"
	norm_est.compute (*normals_out);

}

void visNormals(const PointCloudPtr points,
                        const PointCloudPtr normal_points,
                        const pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::visualization::PCLVisualizer viz,
						string cloudID,
						string normalID,
						string addNormal)
{
  // Add the points and normals to the vizualizer
  viz.addPointCloud (points, cloudID);
  viz.addPointCloud (normal_points, normalID);
  viz.addPointCloudNormals<PointT, pcl::Normal> (normal_points, normals, 1, 0.01, addNormal);
  // Give control over to the visualizer
  viz.spin ();
  viz.removePointCloud(cloudID);
  viz.removePointCloud(normalID);
}

void visKeyPoints (const PointCloudPtr points, const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints, pcl::visualization::PCLVisualizer viz)
{
  // Add the points to the vizualizer
  //pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  // Draw each keypoint as a sphere
  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];
    // Pick the radius of the sphere *
    float r = 0.2 * p.scale;
    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;
    // Add a sphere at the keypoint
    viz.addSphere (p, r, 1.0, 0.0, 0.0, ss.str ());
  }
  // Give control over to the visualizer
  viz.spin ();

}

void downSample(PointCloudPtr &points, PointCloudPtr &points_out)
{
	// down sample...	
	pcl::VoxelGrid<PointT> vox;
	vox.setLeafSize(0.002f, 0.002f, 0.002f);
	vox.setInputCloud(points);
	vox.filter(*points_out);
	
}

void featureCorresp (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());
  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}

void visCorresp(const PointCloudPtr points1,
				const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
				const PointCloudPtr points2,
				const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
				const std::vector<int> &correspondences,
				const std::vector<float> &correspondence_scores,
				pcl::visualization::PCLVisualizer viz)
{
	// We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
	// by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
	// Create some new point clouds to hold our transformed data
	PointCloudPtr points_left (new PointCloud);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
	PointCloudPtr points_right (new PointCloud);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);

	//float angle = M_PI;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.05, 0.0, 0.0; // translate with a zoom in Z-axis
	//transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX())); // x rotation
	pcl::transformPointCloud (*points1, *points_left, transform);
	pcl::transformPointCloud (*points2, *points_right, transform);
	// Shift the first clouds' points to the left
	//const Eigen::Vector3f translate (0.0, 0.0, 0.3);
	//   const Eigen::Vector3f translate (0.4, 0.0, 0.0);
	//   const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	//   pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
	//   pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);
	//   // Shift the second clouds' points to the right
	//   pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
	//   pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

	// Add the clouds to the vizualizer
	viz.addPointCloud (points_left, "points_left");
	viz.addPointCloud (points_right, "points_right");

	// Compute the median correspondence score
	std::vector<float> temp (correspondence_scores);
	std::sort (temp.begin (), temp.end ());
	float median_score = temp[temp.size ()/2];

	// Draw lines between the best corresponding points
	for (size_t i = 0; i < keypoints_left->size (); ++i)
	{
	if (correspondence_scores[i] > median_score)
	{
		continue; // Don't draw weak correspondences
	}
	// Get the pair of points
	const pcl::PointWithScale & p_left = keypoints_left->points[i];
	const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];

	// Generate a random (bright) color
	double r = (rand() % 100);
	double g = (rand() % 100);
	double b = (rand() % 100);
	double max_channel = std::max (r, std::max (g, b));
	r /= max_channel;
	g /= max_channel;
	b /= max_channel;

	// Generate a unique string for each line
	std::stringstream ss ("line");
	ss << i;
	// Draw the line
	viz.addLine (p_left, p_right, r, g, b, ss.str ());

	}
	// Give control over to the visualizer
	viz.spin ();
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

	pcl::PointCloud<pcl::PointWithScale>::Ptr aPC_key (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr bPC_key (new pcl::PointCloud<pcl::PointWithScale>);
	// Creating all basic point clouds
	pcl::PointCloud<pcl::Normal>::Ptr normals_a (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_b (new pcl::PointCloud<pcl::Normal>);
	// Point clouds for PFH descriptors
	pcl::PointCloud<pcl::PFHSignature125>::Ptr desc_a (new pcl::PointCloud<pcl::PFHSignature125>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr desc_b (new pcl::PointCloud<pcl::PFHSignature125>);

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

		// // visualization - adding the inital clouds to the visualizer
		viewer.addPointCloud(aPC, "pca");
		viewer.addPointCloud(bPC, "pcb");
		viewer.spin();
		viewer.removePointCloud("pca");
		viewer.removePointCloud("pcb");

// ---------------------------------------------------------------------------------------------------------------------
//				Trying to detect features using registration
// ---------------------------------------------------------------------------------------------------------------------
		downSample(aPC, aPCSampled);
		downSample(bPC, bPCSampled);
		viewer.addPointCloud(aPC, "pca");
		viewer.addPointCloud(bPC, "pcb");
		viewer.spin();
		viewer.removePointCloud("pca");
		viewer.removePointCloud("pca");

		// first compute the normals
		cout << "computing the normals..." << endl;
		const float normals_radius = 0.03;

		// normals of a
		computeNormals(aPCSampled, normals_radius, normals_a);
		//visNormals(aPC, aPCSampled, normals_a, viewer, "pca_cloud", "pca_normals", "normals_on_pca");
		// // normals of b
		computeNormals(bPCSampled, normals_radius, normals_b);
		//visNormals(bPC, bPCSampled, normals_b, viewer, "pcb_points", "pcb_normals", "normals_on_pcb");
		
		// detect features 
		detectKeyPoints(aPC, aPC_key);
		detectKeyPoints(bPC, bPC_key);

		// PFH features
		const float feature_radius = 0.08;
		computePFHFeatures(aPCSampled, normals_a, aPC_key, feature_radius, desc_a);
  		computePFHFeatures(bPCSampled, normals_b, bPC_key, feature_radius, desc_b);
		
		// Find feature correspondence
		std::vector<int> corresp;
		std::vector<float> corresp_score; // corresp must be above the score to be a correspondence
		featureCorresp(desc_a, desc_b, corresp, corresp_score);
		// quick print to see number of points
		cout << "A has: " << aPC_key->size() << " keypoints out of " << aPCSampled->size() << " total points." << endl;
		cout << "B has: " << bPC_key->size() << " keypoints out of " << bPCSampled->size() << " total points." << endl;
		// Visualize the correspondences between a and b
		visCorresp(aPC, aPC_key, bPC, bPC_key, corresp, corresp_score, viewer);
		

// ---------------------------------------------------------------------------------------------------------------------
		// // visualize keypoints
		// visKeyPoints(aPC, aPC_key, viewer);
		// visKeyPoints(bPC, bPC_key, viewer);
		

		// sift_detect_a.setInputCloud(aPC); // detecting on initial a
		// sift_detect_b.setInputCloud(bPC);

		// sift_detect_a.compute(*aPC_key);// computing a
		// sift_detect_b.compute(*bPC_key); // computing b
		//sift_detect.setInputCloud(bPC);
		//sift_detect.compute(*point_cloud_out_ptr);
		// visualize keypoints
		// viewer.addPointCloud(aPC, "pca"); // display keypoints of first cloud
		// viewer.addPointCloud(bPC, "pcb"); // display keypoints of second cloud
		// cout << aPC_key->size() << "    ehre" << endl;

		// for (size_t x = 0; x < aPC_key->size(); ++x)
    	// {
        // const pcl::PointWithScale &p = aPC_key->points[x];
		// const pcl::PointWithScale &p2 = bPC_key->points[x];
        // float r = 0.2*p.scale;
        // std::stringstream ss ("Keypoint");
        // ss << x;
        // viewer.addSphere(p, r, 1.0, 0.0, 0.0, ss.str());
    	// }
		// viewer.spin();
		//viewer.removePointCloud(aPC, "pca");

		//// down sample...
		// pcl::VoxelGrid<PointT> vox;
		// vox.setLeafSize(0.002f, 0.002f, 0.002f);
		// vox.setInputCloud(aPC);
		// vox.filter(*aPCSampled);
		// vox.setInputCloud(bPC);
		// vox.filter(*bPCSampled);

		// // visualization
		// viewer.addPointCloud(aPCSampled, "pca");
		// viewer.addPointCloud(bPCSampled, "pcb");
		// viewer.spin();
		// viewer.removePointCloud("pca");
    	// viewer.removePointCloud("pcb");

		//// register, moving b to match a.
		// pcl::GeneralizedIterativeClosestPoint6D reg;
		// // we want to transform bPC to match aPC
		// reg.setInputSource(bPCSampled);
		// reg.setInputTarget(aPCSampled);
		// reg.setMaximumIterations(200);
		// reg.setTransformationEpsilon(1e-8);
		// reg.setMaxCorrespondenceDistance(0.1);
		// reg.setEuclideanFitnessEpsilon(0.0000001);
		// // we never actually use outPC...?
		// reg.align(*outPC);

		// Eigen::Matrix4f FunMat = Eigen::Matrix4f::Identity();
		// FunMat = reg.getFinalTransformation();
		// std::cout << "has converged?: " << reg.hasConverged() << ". Score: " << reg.getFitnessScore() << std::endl;
		// std::cout << FunMat << std::endl;

		// pcl::transformPointCloud(*bPC, *bPC, FunMat);

		// viewer.addPointCloud(aPC, "pca");
		// viewer.addPointCloud(bPC, "pcb");
		// viewer.spin();
		// viewer.removePointCloud("pca");
    	// viewer.removePointCloud("pcb");

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
