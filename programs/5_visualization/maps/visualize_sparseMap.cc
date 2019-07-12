#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
	/////////////////////
	// Parse arguments //
	/////////////////////
	
	if (argc != 4)
	{
		std::cerr << "[!] ERROR! The number of expected arguments was 4" << std::endl;
		std::cerr << "  [!] Usage: ./visualize_sparseMap ORBSLAM2_output_directory mapPoints_filename keyFrameTrajectory_filename" << std::endl;

		return 1;
	}

	std::string orbSlam_output_path = argv[1];
	std::string mapPoints_filename = argv[2];
	std::string keyFrames_filename = argv[3];


	//////////////////////////
	// Read sparse map file //
	//////////////////////////
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_map ( new pcl::PointCloud<pcl::PointXYZ> () );
	
	std::string current_frame, id; //Auxiliar variables for reading from files
	float x_3d, y_3d, z_3d; //3D coordinates of each point
	
	//Read source point cloud from the ORB SLAM result
	std::fstream orbMap_file((orbSlam_output_path + mapPoints_filename).c_str());
	while (orbMap_file >> current_frame >> id >> x_3d >> y_3d >> z_3d) //Read lines
	{
		sparse_map->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
	}
	orbMap_file.close();


	//////////////////////////////
	// Read followed trajectory //
	//////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory ( new pcl::PointCloud<pcl::PointXYZ> () );
	
	float q1, q2, q3, q4; //Quaternion representing the rotation of each keyframe
	
	//Read source point cloud from the ORB SLAM result
	std::fstream trajectory_file((orbSlam_output_path + keyFrames_filename).c_str());
	while (trajectory_file >> current_frame >> x_3d >> y_3d >> z_3d >> q1 >> q2 >> q3 >> q4) //Read lines
	{
		trajectory->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
	}
	trajectory_file.close();


	//////////////////////
	// Visualize clouds //
	//////////////////////

	//Viewer
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	//Visualize the sparse map (red)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_sparse (sparse_map, 255, 0, 0);
	viewer.addPointCloud (sparse_map, color_sparse, "sparse_map");

	//Visualize the trajectory (green)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_trajectory (trajectory, 0, 255, 0);
	viewer.addPointCloud (trajectory, color_trajectory, "trajectory");

	//Call the visualization interactor and run internal loop	
	viewer.spin();
	
	return 0;
}

