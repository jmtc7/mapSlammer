#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h> //SOR
#include <pcl/filters/voxel_grid.h> //Voxel grid

namespace fs = boost::filesystem;

// return the filenames of all files that have the specified extension in the specified directory and all subdirectories
void get_all(const fs::path& root, const std::string& name_root, const std::string& ext, std::vector<fs::path>& ret)
{
	if(!fs::exists(root) || !fs::is_directory(root)) return;

	fs::recursive_directory_iterator it(root);
	fs::recursive_directory_iterator endit;

	while(it != endit)
	{
		if(fs::is_regular_file(*it)
			and it->path().extension() == ext
			and it->path().filename().string().find(name_root) != std::string::npos)
		{
			ret.push_back(it->path().filename());
		}

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


	for (int it=0; it<2; it++)
	{
		//If we are in the 2nd iteration
		if(it == 1)
		{
			//Use sparse clouds
			rescaler_nameRoot = rescaler_nameRoot+"SPARSE_";
		}

		std::cout << it << std::endl;
		std::cout << rescaler_nameRoot << std::endl;

		/////////////////////////////////////////////////
		// Get all the .pcd files in the target folder //
		/////////////////////////////////////////////////
	
		std::vector<fs::path> files_list;
		get_all(rescaler_path, rescaler_nameRoot, rescaler_fileExtension, files_list);
	
		std::cout << "[*] Found " << files_list.size() << " '" << rescaler_fileExtension << "' files" << std::endl;
	
	
		/////////////////////
		// Read dense maps //	
		/////////////////////
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr rough_map (new pcl::PointCloud<pcl::PointXYZRGB>);
	
		std::string path_to_pcd;
	
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
	
		for(unsigned int i=0; i<files_list.size(); i++) //For each pointcloud in the target directory
		{
			std::cout << "    [i] Processing cloud #" << i << std::endl;
			//Declare a cloud
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	
			//Read the file
			path_to_pcd = rescaler_path + files_list[i].string();
			pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_to_pcd.c_str(), *cloud);
	
			//Add points to global map
			for (size_t i = 0; i < cloud->points.size(); ++i)
			{
				rough_map->push_back(cloud->points[i]);
			}
		}
	
		//////////////////////
		// Apply voxel grid //
		//////////////////////
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr fine_map (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::VoxelGrid<pcl::PointXYZRGB> voxelg;
		voxelg.setInputCloud (rough_map);
		float voxel_uniform_size = 0.005;
		voxelg.setLeafSize (voxel_uniform_size, voxel_uniform_size, voxel_uniform_size);
		voxelg.filter (*fine_map);
	
		std::cout << "[i] PointCloud after voxel grid: " << fine_map->width * fine_map->height << " data points (" << pcl::getFieldsList (*fine_map) << ").";
	
		sor.setInputCloud (fine_map);
		sor.filter (*fine_map);
	
		//////////////
		// Save map //
		//////////////

		if(it==0)
			pcl::io::savePCDFileASCII ("fine_dense_map.pcd", *fine_map);
		else
			pcl::io::savePCDFileASCII ("fine_sparse_map.pcd", *fine_map);

		std::cout << "[*] Saved " << fine_map->points.size () << " data points" << std::endl;
	}

	return 0;
}

