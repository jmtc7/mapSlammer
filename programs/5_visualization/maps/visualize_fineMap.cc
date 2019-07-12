#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
	//Read pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fine_map ( new pcl::PointCloud<pcl::PointXYZRGB> () );

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("fine_dense_map.pcd", *fine_map) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read fine map \n");
		return (-1);
	}

	//Read followed trajectory (GT)
	pcl::PointCloud<pcl::PointXYZ>::Ptr trajectory ( new pcl::PointCloud<pcl::PointXYZ> () );

	std::string current_frame;	
	float x_3d, y_3d, z_3d, q1, q2, q3, q4; //Quaternion representing the rotation of each keyframe
	
	//Read source point cloud from the ORB SLAM result
	std::fstream trajectory_file("/home/w4nd3r/tfg/aaa_path/v4_full/results/fr2/desk/output_files/ORB_SLAM2/KeyFrameTrajectory.txt");
	while (trajectory_file >> current_frame >> x_3d >> y_3d >> z_3d >> q1 >> q2 >> q3 >> q4) //Read lines
	{
		trajectory->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
	}
	trajectory_file.close();


	


	//Visualize sparse map in the viewer
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Map viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fine_map);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb (fine_map, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (fine_map, rgb, "fine_map");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (trajectory, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ> (trajectory, green, "trajectory");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "trajectory");

	viewer->setCameraPosition (-1.10823,0.0290256,-1.35437, //location
				0.0851341,-0.993809,-0.0713868); //view up direction 

	//Call the visualization interactor and run internal loop	
	viewer->spin();

	return 0;
}

