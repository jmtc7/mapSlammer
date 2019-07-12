#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>
#include <pcl/filters/statistical_outlier_removal.h> //SOR


namespace fs = boost::filesystem;

// return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
void get_all(const fs::path& root, const std::string& ext, std::vector<fs::path>& ret)
{
	if(!fs::exists(root) || !fs::is_directory(root)) return;

	fs::recursive_directory_iterator it(root);
	fs::recursive_directory_iterator endit;

	while(it != endit)
	{
		if(fs::is_regular_file(*it) && it->path().extension() == ext)
			ret.push_back(it->path().filename());
		++it;
	}
}


int main (int argc, char** argv)
{

	if (argc != 6)
	{
		std::cerr << "[!] ERROR! The number of expected arguments was 6" << std::endl;
		std::cerr << "  [!] Usage: ./visualize_denseMap ORBSLAM2_output_directory mapPoints_filename rescaler_output_directory rescaledPointclouds_filename_root rescaledPointclouds_filename_extension" << std::endl;

		return 1;
	}

	//Parse arguments
	fs::path orbSlam_output_path = argv[1];
	std::string mapPoints_filename = argv[2];
	std::string rescaler_path = argv[3];
	std::string rescaler_nameRoot = argv[4];
	std::string rescaler_fileExtension = argv[5];

	//Read all the .pcd files in the target folder
	std::vector<fs::path> files_list;
	get_all(rescaler_path, rescaler_fileExtension, files_list);

	std::cout << "[*] Found " << files_list.size() << " '" << rescaler_fileExtension << "' files" << std::endl;
	
	//Visualize pointclouds
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	std::string path_to_pcd;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr map (new pcl::PointCloud<pcl::PointXYZRGB>);

	for(unsigned int i=0; i<files_list.size(); i++) //For each pointcloud in the target directory
	{
		//Declare a cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	
		//Read the file
		path_to_pcd = rescaler_path + files_list[i].string();
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_to_pcd.c_str(), *cloud);

		for (unsigned int j=0; j<cloud->points.size(); j++)
		{
			map->push_back(cloud->points[j]);
		}		
	}

	//Read sparse map file
	pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_map ( new pcl::PointCloud<pcl::PointXYZ> () );
	
	std::string current_frame, id; //Auxiliar variables for reading from files
	float x_3d, y_3d, z_3d; //3D coordinates of each point
	
	//Read source point cloud from the ORB SLAM result
	std::fstream orbMap_file((orbSlam_output_path.string() + mapPoints_filename).c_str());
	while (orbMap_file >> current_frame >> id >> x_3d >> y_3d >> z_3d) //Read lines
	{
		//TODO - When the current frame has been processed -> Break the loop!
		sparse_map->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
	}
	orbMap_file.close();


	//////////////
	// SAVE MAP //
	//////////////
	if (map->size() != 0)
	{
		pcl::io::savePCDFileASCII ("full_demon_dense.pcd", *map);
		std::cout << "[*] Saved " << map->points.size () << " data points" << std::endl;
	}
	else
	{
		std::cerr << "[!] ERROR! No data in the map"  << std::endl;
	}


	///////////////
	// VISUALIZE //
	///////////////
	
	//Visualize dense map in the viewer
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(map);
	viewer.addPointCloud (map, rgb, "dense_map");// note that before it was showCloud

	//Visualize sparse map in the viewer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_sparse (sparse_map, 0, 255, 0);
	viewer.addPointCloud (sparse_map, color_sparse, "sparse_map");// note that before it was showCloud
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sparse_map"); //Increase points size

	//Call the visualization interactor and run internal loop	
	viewer.spin();
	
	return 0;
}

