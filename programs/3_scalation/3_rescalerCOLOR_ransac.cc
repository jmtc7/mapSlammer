//TODO - Implement general enaugh function to read both mappoints and demon estimations from raw txt file

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <cmath> //To raise numbers with "pow()"

//Convert a transformation matrix to a quaternion (from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/)
std::vector<float> get_quat_from_mat(Eigen::Matrix<float,4,4> trans_mat)
{
	std::vector<float> quaternion;
	float qx, qy, qz, qw;

	//Extract data from matrix (to clarify the calculations)
	float m00, m01, m02;
	float m10, m11, m12;
	float m20, m21, m22;

	m00 = trans_mat(0, 0);
	m01 = trans_mat(0, 1);
	m02 = trans_mat(0, 2);
	
	m10 = trans_mat(1, 0);
	m11 = trans_mat(1, 1);
	m12 = trans_mat(1, 2);

	m20 = trans_mat(2, 0);
	m21 = trans_mat(2, 1);
	m22 = trans_mat(2, 2);

	//Calculate quaternion using the transformation matrix
	float tr = m00 + m11 + m22;

	if (tr > 0)
	{ 
		float S = sqrt(tr + 1.0) * 2; // S=4*qw 
		qw = 0.25 * S;
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S; 
		qz = (m10 - m01) / S; 
	}
	else if ((m00 > m11)&(m00 > m22))
	{ 
		float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		qw = (m21 - m12) / S;
		qx = 0.25 * S;
		qy = (m01 + m10) / S; 
		qz = (m02 + m20) / S; 
	}
	else if (m11 > m22)
	{ 
		float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		qw = (m02 - m20) / S;
		qx = (m01 + m10) / S; 
		qy = 0.25 * S;
		qz = (m12 + m21) / S; 
	}
	else
	{ 
		float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		qw = (m10 - m01) / S;
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;
	}

	//Assign values to return vector and end
	quaternion.push_back(qx);
	quaternion.push_back(qy);
	quaternion.push_back(qz);
	quaternion.push_back(qw);

	return quaternion;
}

//Convert the matrix to a transformation string
std::string get_transf_from_mat(Eigen::Matrix<float,4,4> full_demon_trans)
{
	std::ostringstream trans_str;

	//get translation from the matrix ((0,3), (1,3), (2,3) idxs);
	float tx, ty, tz;
	tx = full_demon_trans(0, 3);
	ty = full_demon_trans(1, 3);
	tz = full_demon_trans(2, 3);

	//transform matrix to quaternion;
	float qx, qy, qz, qw;
	std::vector<float> quat = get_quat_from_mat(full_demon_trans);

	qx = quat[0];
	qy = quat[1];
	qz = quat[2];
	qw = quat[3];

	//build string (tx ty tz q1 q2 q3 q4)
	trans_str << tx << " " << ty << " " << tz << " "
		  << qx << " " << qy << " " << qz << " " << qw;

	return trans_str.str();
}      


float compute_error(Eigen::Matrix<float,4,4> transformation, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float threshold)
{
	std::vector<float> distances;
	float n_inliers = 0;

	//Transform target cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud (*cloud_target, *transformed_target, transformation);

	//Compute distances between each pair of points
	for (int i=0; i<cloud_source->points.size(); i++)
	{
		//Compute distance between points  -->  sqrt( (x'-x)^2 + (y'-y)^2 + (z'-z)^2 )
		float distance = sqrt(
				std::pow((transformed_target->points[i].x - cloud_source->points[i].x), 2) +
				std::pow((transformed_target->points[i].y - cloud_source->points[i].y), 2) +
				std::pow((transformed_target->points[i].z - cloud_source->points[i].z), 2)
				); 

		//Push distance to vector
		distances.push_back(distance);
	}

	//Sum the distances below the threshold(the ones from the inliers)
	for (int i=0; i<distances.size(); i++)
	{
		if(distances[i] < threshold)
		{
			n_inliers++; //Count one inlier
		}
	}

	return n_inliers; //Return the number of inliers
}



int main (int argc, char** argv)
{
	if (argc != 11)
	{
		std::cerr << "[!] ERROR! The number of expected arguments was 11" << std::endl;
		std::cerr << "  [!] Usage: ./3_rescaler ORBSLAM2_output_directory keyFrame_filename mapPoints_filename depthEstimation_output_directory depthEstimation_filename_root FULL_depthEstimation_filename_root depthEstimation_filename_extension path_to_save_outputs rescaledPointclouds_filename_root rescaledPointclouds_filename_extension" << std::endl;

		return 1;
	}


	//Parse arguments
	std::string orbSlam_output_path = argv[1];
	std::string keyFrameTrajectory_filename = argv[2];
	std::string mapPoints_filename = argv[3];
	std::string depthEstimation_path = argv[4];
	std::string depthEstimation_nameRoot = argv[5];
	std::string depthEstimation_full_nameRoot = argv[6];
	std::string depthEstimation_fileExtension = argv[7];
	std::string outputs_path = argv[8];
	std::string outputs_nameRoot = argv[9];
	std::string outputs_fileExtension = argv[10];


	//Open trajectory file (ORB-SLAM2 output) and use it as target frame list
	std::fstream frame_list((orbSlam_output_path+keyFrameTrajectory_filename).c_str());
	std::string current_frame;
	float x, y, z, q1, q2, q3, q4;

	//Open transformations file (demon_transformations.txt)
	std::ofstream trans_file;
	trans_file.open(std::string(outputs_path + "ransac_transformations.txt").c_str());

	while(frame_list >> current_frame >> x >> y >> z >> q1 >> q2 >> q3 >> q4) //For each keyframe in the keyframe list...
	{
		//TODO - Create 4 clouds just one time and delete their content in every loop iteration

		//////////////////////////////////////////////////
		// Read pointclouds (MPs, DeMoN and DeMoN_FULL) //
		//////////////////////////////////////////////////

		//Declare pointclouds	
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_target ( new pcl::PointCloud<pcl::PointXYZRGB> () );

		//Auxiliar variables for reading from files
		std::string name, id;
		float x_3d, y_3d, z_3d; //3D coordinates of each point
		int r, g, b; //Color values for the "full target"

		//Read source point cloud from the ORB SLAM result
		std::fstream orbMap_file((orbSlam_output_path+mapPoints_filename).c_str());
		while (orbMap_file >> name >> id >> x_3d >> y_3d >> z_3d) //Read lines
		{
			//TODO - When the current frame has been processed -> Break the loop!

			if(name == current_frame)
				cloud_source->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
		}
		orbMap_file.close();

		//Read target point cloud from the DeMoN result
		std::fstream demon3d_file((depthEstimation_path + depthEstimation_nameRoot + current_frame + depthEstimation_fileExtension).c_str());
		while (demon3d_file >> x_3d >> y_3d >> z_3d) //Read lines
		{
			cloud_target->push_back (pcl::PointXYZ (x_3d, y_3d, z_3d)); //Add data to pointcloud
		}
		demon3d_file.close();

		if (cloud_target->points.size() == 0)
		{
			//TODO - DO NOT read the last line of the KeyFrameTrajectory.txt file, to avoid getting this error 

			std::cerr << "[!] ERROR! There are no estimated points for the frame " << current_frame << std::endl;
			std::cerr << "  [i] Ending the workflow..." << std::endl;
			break;
		}

		//Read FULL point cloud from the DeMoN result
		std::fstream full_demon3d_file((depthEstimation_path + depthEstimation_full_nameRoot + current_frame + depthEstimation_fileExtension).c_str());
		while (full_demon3d_file >> x_3d >> y_3d >> z_3d >> r >> g >> b) //Read lines
		{
			pcl::PointXYZRGB point;
			point.x = x_3d;
			point.y = y_3d;
			point.z = z_3d;
			point.r = r;
			point.g = g;
			point.b = b;

			full_target->push_back (point); //Add data to pointcloud
		}
		full_demon3d_file.close();


		/////////////////////////////////////////////////////////
		// Get scale using SVD as a model generator for RANSAC //
		/////////////////////////////////////////////////////////
		int ransac_iterations = 300; //Number of RANSAC iterations
		int matches_quantity = 10; //Number of point pairs will be randomly taken
		float error_threshold = 0.02; //Threshold to consider points inliers (after applying the transformation)
		int min_inliers = int(cloud_source->points.size()*0.25);

		pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ, pcl::PointXYZ, float> svd; //Transformation estimator
		Eigen::Matrix<float,4,4> transformation;

		float max_inliers = 0; //Result error
		Eigen::Matrix<float,4,4> best_transformation = Eigen::Matrix4f::Identity(); //4x4 identity matrix
		Eigen::Matrix<float,4,4> best_refination = Eigen::Matrix4f::Identity(); //4x4 identity matrix


		//Use current time as the seed to generate random numbers (indices of the cloud)
		srand(time(NULL));

		//RANSAC 
		for (int ri=0; ri<ransac_iterations; ri++)
		{
			std::vector<int> indices;

			//Select random point pairs
			for (int j=0; j<matches_quantity;  j++)
			{
				int random_idx = rand() % cloud_source->points.size();
				//TODO - Check if already pushed (not done bc low probability and a lot of iterations...)
				indices.push_back(random_idx);
			}

			//Get transformation between point pairs
			svd.estimateRigidTransformation(*cloud_target, indices, *cloud_source, indices, transformation);
			transformation(0,3) = 0.0;
			transformation(1,3) = 0.0;
			transformation(2,3) = 0.0;

			//Apply rough transformation (without translation) in an auxiliar cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr aux_target ( new pcl::PointCloud<pcl::PointXYZ> () );
			pcl::transformPointCloud (*cloud_target, *aux_target, transformation); //Apply transformation

			//Get refined transformation (mostly, only translation)
			Eigen::Matrix<float,4,4> refination;
			svd.estimateRigidTransformation(*aux_target, indices, *cloud_source, indices, refination); //Refine

			//Count inliers (using the full clouds (ORB and DeMoN sparse))
			float n_inliers = compute_error(refination, cloud_source, aux_target, error_threshold);

			//Check if a better solution has been found
			if (n_inliers > max_inliers)
			{
				max_inliers = n_inliers;
				best_transformation = transformation;
				best_refination = refination;
			}

		} //RANSAC

		if (max_inliers < min_inliers) //If no inliers were archived
		{
			std::cout << "[!] ERROR! No valid transformation found for: " << current_frame << std::endl;
			std::cout << "[i] Inliers: " << max_inliers << "/" << cloud_source->points.size() << std::endl;
		}
		else
		{
			//Write transformation (keyframe + translation + quaternion)
			Eigen::Matrix<float,4,4> full_demon_trans;
			full_demon_trans = best_refination*best_transformation;

			std::string transf = get_transf_from_mat(full_demon_trans); //Convert the matrix to a transformation string

			trans_file << current_frame << " " << transf << std::endl; //Write in file


			//Print obtained transformation
			std::cout << "[i] SUCCESS! Valid transformation found for: " << current_frame << std::endl;
			std::cout << "[i] Inliers: " << max_inliers << "/" << cloud_source->points.size() << std::endl;

			/////////////////////////////////	
			// Apply scale (to DeMoN_FULL) //
			/////////////////////////////////
			//TODO - Estimate an extra refination using the inliers of the best transformation?
			pcl::transformPointCloud (*cloud_target, *cloud_target, best_transformation);
			pcl::transformPointCloud (*cloud_target, *cloud_target, best_refination);

			pcl::transformPointCloud (*full_target, *full_target, best_transformation);
			pcl::transformPointCloud (*full_target, *full_target, best_refination);

			///////////////////////////////////////////////
			// Save the rescaled pointcloud (DeMoN_FULL) //
			///////////////////////////////////////////////
			if (full_target->size() != 0)
			{
				pcl::io::savePCDFileASCII (outputs_path+outputs_nameRoot+current_frame+outputs_fileExtension, *full_target);
				std::cout << "    [*] Saved " << full_target->points.size () << " data points to " + outputs_path + outputs_nameRoot + current_frame + outputs_fileExtension << std::endl;
			}
			else
			{
				std::cerr << "    [!] ERROR! No [FULL] data for frame " << current_frame << std::endl;
			}
			
			if (cloud_target->size() != 0)
			{
				pcl::io::savePCDFileASCII (outputs_path+outputs_nameRoot+"SPARSE_"+current_frame+outputs_fileExtension, *cloud_target);
				std::cout << "    [*] Saved " << cloud_target->points.size () << " data points to " + outputs_path + outputs_nameRoot + current_frame + outputs_fileExtension << std::endl;
			}
			else
			{
				std::cerr << "    [!] ERROR! No [SPARSE] data for frame " << current_frame << std::endl;
			}
		}

	}

	//Close KeyFrameTrajectory and transformation file
	frame_list.close();
	trans_file.close();

	return 0;
}

