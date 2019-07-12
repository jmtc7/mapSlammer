#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
	/////////////////////
	// Parse arguments //
	/////////////////////
	
	if (argc != 7)
	{
		std::cerr << "[!] ERROR! The number of expected arguments was 7" << std::endl;
		std::cerr << "  [!] Usage: ./visualize_demon ORBSLAM2_output_directory keyFrameTrajectory_filename mapPoints_filename demon_path demon_fileRoot demon_fileExtension" << std::endl;

		return 1;
	}

	std::string orbSlam_output_path = argv[1];
	std::string keyFrameTrajectory_filename = argv[2];
	std::string mapPoints_filename = argv[3];
	std::string depthEstimation_path = argv[4];
	std::string depthEstimation_nameRoot = argv[5];
	std::string depthEstimation_fileExtension = argv[6];

	//Open trajectory file (ORB-SLAM2 output) and use it as target frame list
	std::fstream frame_list((orbSlam_output_path + keyFrameTrajectory_filename).c_str());
	std::string current_frame;
	float x, y, z, q1, q2, q3, q4, x_3d, y_3d, z_3d;

	pcl::PointCloud<pcl::PointXYZ>::Ptr demon_map ( new pcl::PointCloud<pcl::PointXYZ> () );
	std::fstream demon3d_file;
	while(frame_list >> current_frame >> x >> y >> z >> q1 >> q2 >> q3 >> q4) //For each keyframe in the keyframe list...
	{
		//Read target point cloud from the DeMoN result
		demon3d_file.open((depthEstimation_path + depthEstimation_nameRoot + current_frame + depthEstimation_fileExtension).c_str());
		while (demon3d_file >> x_3d >> y_3d >> z_3d) //Read lines
		{
			demon_map->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
		}
		demon3d_file.close();

	}
	frame_list.close();


	//////////////////////////
	// Read sparse map file //
	//////////////////////////
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_map ( new pcl::PointCloud<pcl::PointXYZ> () );
	
	std::string id; //Auxiliar variables for reading from files
	
	//Read source point cloud from the ORB SLAM result
	std::fstream orbMap_file((orbSlam_output_path + mapPoints_filename).c_str());
	while (orbMap_file >> current_frame >> id >> x_3d >> y_3d >> z_3d) //Read lines
	{
		sparse_map->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
	}
	orbMap_file.close();


	//////////////////////
	// Visualize clouds //
	//////////////////////

	//Viewer
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	//Visualize the demon map (red)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_map (demon_map, 255, 0, 0);
	viewer.addPointCloud (demon_map, color_map, "demon_maps");

	//Visualize the sparse map (green)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_sparse (sparse_map, 0, 255, 0);
	viewer.addPointCloud (sparse_map, color_sparse, "sparse_map");

	//Call the visualization interactor and run internal loop	
	viewer.spin();
	
	return 0;
}

