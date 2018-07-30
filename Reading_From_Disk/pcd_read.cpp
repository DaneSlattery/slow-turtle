#include <iostream>
#include <pcl/io/pcd_io.h> // Allows for .pcd to be read from disk
//#include <pcl/io/ply_io.h>  // Allows for .ply to be read from disk
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  // create pointcloud boost shared pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPLYFile<pcl::PointXYZ> ("cube.ply", *cloud) == -1) //* load PLY file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("wolf.pcd", *cloud) == -1) //* load PCD file
  {
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }
  std::cout << "Loaded " << cloud->width * cloud->height << "datapoints" << std::endl;
  // Shows the data that was loaded          
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

  return (0);
}