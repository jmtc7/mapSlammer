#!/bin/bash

#####################################################
####  G E N E R A L   I N F O   P R I N T I N G  ####
#####################################################

#Clean the screen of previous text
clear

#Show some general information about this bashscript
echo "
#########################################################################################################
 ##########################################################################################################
  #####  ███╗   ███╗ █████╗ ██████╗     ███████╗██╗      █████╗ ███╗   ███╗███╗   ███╗███████╗██████╗   ####
   ####  ████╗ ████║██╔══██╗██╔══██╗    ██╔════╝██║     ██╔══██╗████╗ ████║████╗ ████║██╔════╝██╔══██╗   ####
    ###  ██╔████╔██║███████║██████╔╝    ███████╗██║     ███████║██╔████╔██║██╔████╔██║█████╗  ██████╔╝    #####
    ###  ██║╚██╔╝██║██╔══██║██╔═══╝     ╚════██║██║     ██╔══██║██║╚██╔╝██║██║╚██╔╝██║██╔══╝  ██╔══██╗    #####
   ####  ██║ ╚═╝ ██║██║  ██║██║         ███████║███████╗██║  ██║██║ ╚═╝ ██║██║ ╚═╝ ██║███████╗██║  ██║   ####
  #####  ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝         ╚══════╝╚══════╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝  ####
 ##########################################################################################################
#########################################################################################################

Description
-----------
An automated workflow for creating dense 3D maps and reconstructinos using monocular data. It solves the
scale issue on DeMoN using an ORB-SLAM2 generated sparse map and densifies this map using the DeMoN's
depth estimations.

This project consists in a method to create a dense three-dimensional representations of the environment
in which a single 2D camera is moving with an unconstrained movenet. This is done by fusing the output of
a keyframe-based visual SLAM algorithm (ORB-SLAM2) with predicted depthmaps obtained with a Deep 
Learning-based method (DeMoN).

The current results provide maps that densify the ORB-SLAM2 output extracting almost 200 times more data
from each keyframe (49152 vs 250.54 3D points). The resulting map's average distance to the nearest neighbours
of the groundtruths is 0.0399, with a variance of 0.0006. This measures have been computed using various
sequences from the TUM RGB-D SLAM dataset.


Glossary
--------
This script uses the next information symbols:
    - [>] Script asking for user input
    - [i] Information
    - [!] Error
"




#####################################
####  G E N E R A L   S E T U P  ####
#####################################
#TODO - Ask the user if he just wants to visualize the results
#TODO - Parametrize the user asking processess and put them in a single function/include file -> https://ryanstutorials.net/bash-scripting-tutorial/bash-functions.php
#TODO - By default, everything is "yes". Take in account "y", "Y", "n" and "N" everywhere.

echo "
GENERAL SETUP PROCESS
====================="

#Path where the dataset is stored
#def_images_directory="sample_sourceDataset/rgbd_dataset_freiburg1_xyz/" #xyz dataset
#def_images_directory="sample_sourceDataset/rgbd_dataset_freiburg1_360/" #360 dataset
#def_images_directory="sample_sourceDataset/rgbd_dataset_freiburg1_desk/" #Desk1 dataset
#def_images_directory="sample_sourceDataset/rgbd_dataset_freiburg1_desk2/" #Desk2 dataset
#def_images_directory="sample_sourceDataset/rgbd_dataset_freiburg2_xyz/" #xyz dataset (FR2)
def_images_directory="sample_sourceDataset/rgbd_dataset_freiburg2_desk/" #Desk1 dataset (FR2)

#def_images_directory="sample_sourceDataset/rgb_corridor_kinect/" #xyz dataset

images_directory=$def_images_directory

#File where useless outputs from some processes will be throwed
useless_outputs_logfile_name="useless_outputs.log" 


#Ask the user if he just wants to visualize data
read -p "[>] Are you here just to visualize results? (yes/no): " just_visualize




############################################
####  O R B - S L A M  2   O U T P U T  ####
############################################
#TODO - Try to give an option of not to invest computational resources in a graphical visualization of the algorithm

echo "

GET ORB-SLAM2 OUTPUT
===================="

#ORB-SLAM2's output files names
keyFrame_file="KeyFrameTrajectory.txt"
keyPoints_file="KeyPoints.txt"
mapPoints_file="MapPoints.txt"

#Path in which the output files will be saved in
def_source_directory="output_files/ORB_SLAM2/"
source_directory=$def_source_directory

#Path to the camera parameters and general ORB settings
def_settings_path="programs/ORB_SLAM2/settings/TUM1.yaml"
#def_settings_path="sample_sourceDataset/rgb_corridor_kinect/kinect.yaml"
settings_path=$def_settings_path

#Path to the vocabulary used for the BoW
def_vocabulary_path="programs/ORB_SLAM2/vocabulary/ORBvoc.txt"
vocabulary_path=$def_vocabulary_path

#Set logfile name
slam_logfile_name=0_ORB_SLAM2.log

if test $just_visualize == "no"
then
	#Ask if the user wants to use already generated data (from ORB-SLAM2)
	read -p "[>] Would you like to use already generated ORB-SLAM2 output? (yes/no): " use_generated_source
	
	#Localize source data (or generate it)
	if test $use_generated_source == "yes"
	then
		#Get path to source directory
		echo    "    [i] Great! We will work with a pre-generated output!"
		read -p "    [>] Please, introduce the ORB-SLAM2 output's global path (default: $def_source_directory): " source_directory
	
		#Check if the user wanted the default source directory (he did not entered any input)
		if test "$source_directory" == ""
		then
			source_directory=$def_source_directory
			echo "        [i] No input detected! Using '$source_directory' as source data directory"
		else
			#Copy the output from the given folder to this workspace
			cp $source_directory$keyFrame_file $def_source_directory
			cp $source_directory$keyPoints_file $def_source_directory
			cp $source_directory$mapPoints_file $def_source_directory
			
			source_directory=$def_source_directory
		fi
	
	else
		#Get source images path
		echo    "    [i] OK! Then I'll need the source images..."
		read -p "    [>] Please, introduce the input images's global path (default: $def_images_directory): " images_directory
		
		#Check if the user wanted the default dataset (he did not entered any input)
		if test "$images_directory" == ""
		then
			images_directory=$def_images_directory
			echo "        [i] No input detected! Using '$images_directory' as dataset directory"
		fi
	
		#Get settings file path
		read -p "    [>] Are this images form a TUM1 dataset? (yes/no): " tum_dataset
		if test $tum_dataset == "yes"
		then
			echo "        [i] Great! I already have the camera and ORB-SLAM2 settings!"
			settings_path=$def_settings_path
		else
			echo    "        [i] OK... Then I'll need the camera parameters and ORB SLAM 2 settings..."
			read -p "        [>] Please, introduce the global path to the settings file: " settings_path
		fi
		
		#Offer the possibility to use a custom vocabulary
		read -p "    [>] Do you want to use the default vocabulary? (yes/no): " use_def_voc
		if test $use_def_voc == "yes"
		then
			echo "        [i] Great! I already have the default vocabulary!"
			vocabulary_path=$def_vocabulary_path
		else
			echo    "        [i] OK... Then I'll need an alternative vocabulary..."
			read -p "        [>] Please, introduce the global path to the vocabulary file: " vocabulary_path
		fi
	
		#Execute ORB-SLAM2 (telling the path to the vocabulary, to the settings and to the dataset, alongside with where to save the outputs and their names)
		rm $def_source_directory* &>logs/$useless_outputs_logfile_name #Remove previous results (closing stdoutput and stderr)
		rm logs/$slam_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
		echo "    [i] Running ORB-SLAM2 over the given dataset... This might take a while"
		./programs/ORB_SLAM2/Examples/Monocular/mono_tum $vocabulary_path $settings_path $images_directory $source_directory $keyFrame_file $mapPoints_file $keyPoints_file &>logs/$slam_logfile_name #2>logs/$useless_outputs_logfile_name
	
		#Evaluate exit status of the ORB-SLAM2 execution
		if test $? == 0
		then
			echo "[i] Done! Files saved to 'output_files/ORB_SLAM2/'" 
		else
			echo "[!] ERROR! Unknown error happened while executing ORB-SLAM2"
			echo "[i] Exiting the workflow..."
			exit
		fi
	fi
else
	echo "[i] Omitted, the user just wants to visualize data"
fi

#Set the source_directory to the one in this workspace
source_directory=$def_source_directory






###############################################################
####  3 D   M A P   P O I N T S   R E P R O J E C T I O N  ####
###############################################################
#TODO - NOT to have two removal processes...

echo "

REPROJECT 3D MAP POINTS INTO ITS IMAGE PLANE
============================================"

#Set output directory
def_reprojection_directory="output_files/1_reprojection/"
reprojection_directory=$def_reprojection_directory

#Set output filename's root - "reprojecter.py" will add the framename and the extension
reprojectedKeyPoints_filename_root="reprojectedKeyPoints_"

#Set output filename's extension
reprojectedKeyPoints_filename_extension=".txt"

#Set logfile name
reproj_logfile_name=1_reprojecter.log


if test $just_visualize == "no"
then
	#Ask if the user wants to use already reprojected points
	read -p "[>] Would you like to use already reprojected keypoints? (yes/no): " use_generated_reprojKPs
	
	#Localize reprojected kepoints (or generate them)
	if test $use_generated_reprojKPs == "yes"
	then
		#Get path to source directory
		echo    "    [i] Great! We will work with pre-reprojected points!"
		read -p "    [>] Please, introduce the reprojected points's global path (default: $def_reprojection_directory): " reprojection_directory
	
		#Check if the user wanted the default source directory (he did not entered any input)
		if test "$reprojection_directory" != ""
		then
			#Remove old reprojections
			rm $def_reprojection_directory* &>logs/$useless_outputs_logfile_name #Remove previous reprojections (closing outputs)
			rm logs/$reproj_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
	
			#Copy the reprojections from the given folder to this workspace
			cp $reprojection_directory$reprojectedKeyPoints_filename_root* $def_reprojection_directory
		else
			reprojection_directory=$def_reprojection_directory
			echo "        [i] No input detected! Using '$reprojection_directory' as the reprojected points directory"
		fi
	else
		#Remove old reprojections
		rm $def_reprojection_directory* &>logs/$useless_outputs_logfile_name #Remove previous reprojections (closing outputs)
		rm logs/$reproj_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
	
		echo "    [i] Reprojecting the map points in the corresponding 2D planes..."
		#Reproject, telling where does the KeyFrames list is and where to save the reprojection results
		python3 programs/1_keypointsReprojection/1_reprojecter.py $source_directory $keyFrame_file $mapPoints_file $reprojection_directory $reprojectedKeyPoints_filename_root $reprojectedKeyPoints_filename_extension &>logs/$reproj_logfile_name
		#python3 programs/1_keypointsReprojection/1_reprojecter_filtered.py $source_directory $keyFrame_file $mapPoints_file $reprojection_directory $reprojectedKeyPoints_filename_root $reprojectedKeyPoints_filename_extension &>logs/$reproj_logfile_name
	fi	
else
	echo "[i] Omitted, the user just wants to visualize data"
fi






###############################################################
####  D E N S E   D E P T H   M A P   E S T I M A T I O N  ####
###############################################################

echo "

ESTIMATE DENSE DEPTH MAPS
========================="

#Set output directory
def_depthEstimation_directory="output_files/2_depthEstimation/"
depthEstimation_directory=$def_depthEstimation_directory

#Set output filename's root (reduced pointclouds) - "depthEstimator.py" will add the framename and the extension
depthEstimation_filename_root="demonPoints_"

#Set output filename's root (full pointclouds) - "depthEstimator.py" will add the framename and the extension
depthEstimation_FULL_filename_root="demonPoints_FULL_"

#Set output filename's extension
depthEstimation_filename_extension=".txt"

#Set logfile name
depthEstimation_logfile_name=2_depthEstimation.log

if test $just_visualize == "no"
then
	#Ask if the user wants to use already estimated depths
	read -p "[>] Would you like to use already estimated depths? (yes/no): " use_estimated_depths
	
	#Localize estimated depths (or generate them)
	if test $use_estimated_depths == "yes"
	then
		#Get path to source directory
		echo    "    [i] Great! We will work with pre-estimated depths!"
		read -p "    [>] Please, introduce the estimated depths's global path (default: $def_depthEstimation_directory): " depthEstimation_directory
	
		#Check if the user wanted the default source directory (he did not entered any input)
		if test "$depthEstimation_directory" != ""
		then
			#Remove old reprojections
			rm $def_depthEstimation_directory* &>logs/$useless_outputs_logfile_name #Remove previous depth estimations (closing outputs)
			rm logs/$depthEstimation_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
	
			#Copy the reprojections from the given folder to this workspace
			cp $depthEstimation_directory$depthEstimation_filename_root* $def_depthEstimation_directory 
			cp $depthEstimation_directory$depthEstimation_FULL_filename_root* $def_depthEstimation_directory 
		else
			depthEstimation_directory=$def_depthEstimation_directory 
			echo "        [i] No input detected! Using '$depthEstimation_directory' as the estimated depths directory"
		fi
	else
		#Remove old reprojections
		rm $def_depthEstimation_directory* &>logs/$useless_outputs_logfile_name #Remove previous depth estimations (closing outputs)
		rm logs/$depthEstimation_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
	
		echo "    [i] Estimating depthmaps... This might take a while"
		
		#Estimate the depth
		python3 programs/2_depthEstimation/demon/examples/2_depthEstimatorCOLOR.py ../../../../$images_directory/rgb/ $source_directory $keyFrame_file $reprojection_directory $reprojectedKeyPoints_filename_root $reprojectedKeyPoints_filename_extension $depthEstimation_directory $depthEstimation_filename_root $depthEstimation_FULL_filename_root $depthEstimation_filename_extension &>logs/$depthEstimation_logfile_name #STORE COLOR OF DENSE FULL CLOUD
		#python3 programs/2_depthEstimation/demon/examples/2_depthEstimatorCOLOR_filtered.py ../../../../$images_directory/rgb/ $source_directory $keyFrame_file $reprojection_directory $reprojectedKeyPoints_filename_root $reprojectedKeyPoints_filename_extension $depthEstimation_directory $depthEstimation_filename_root $depthEstimation_FULL_filename_root $depthEstimation_filename_extension &>logs/$depthEstimation_logfile_name #STORE COLOR OF DENSE FULL CLOUD
	fi	
else
	echo "[i] Omitted, the user just wants to visualize data"
fi






#######################################################
####  D E M O N   R E S U L T   S C A L A T I O N  #### 
#######################################################
#TODO - Allow the user to choose if the resulting clouds are filtered (using a passThrough in the Z axis)

echo "

SCALE THE DEPTH ESTIMATIONS
==========================="

#Set output directory
def_scalation_directory="output_files/3_scalation/"
scalation_directory=$def_scalation_directory

#Set output filename's root
pointcloud_filename_root="pointcloud_"

#Set output filename's extension
pointclouds_filename_extension=".pcd"

#Set logfile name
rescale_logfile_name=3_rescaler.log

if test $just_visualize == "no"
then
	#Ask if the user wants to use already estimated depths
	read -p "[>] Would you like to use already escalated clouds? (yes/no): " use_escalated_clouds
	
	#Localize estimated depths (or generate them)
	if test $use_escalated_clouds == "yes"
	then
		#Get path to source directory
		echo    "    [i] Great! We will work with pre-rescaled clouds!"
		read -p "    [>] Please, introduce the rescaled clouds's global path (default: $def_scalation_directory): " scalation_directory
	
		#Check if the user wanted the default scalation directory (he did not entered any input)
		if test "$scalation_directory" != ""
		then
			#Remove old scalations
			rm $def_scalation_directory* &>logs/$useless_outputs_logfile_name #Remove previous scalations (closing outputs)
			rm logs/$rescale_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
	
			#Copy the reprojections from the given folder to this workspace
			cp $scalation_directory$pointcloud_filename_root* $def_scalation_directory 
		else
			scalation_directory=$def_scalation_directory 
			echo "        [i] No input detected! Using '$scalation_directory' as the rescaled pointclouds directory"
		fi
	else
		#Remove old scalatoins
		rm $def_scalation_directory* &>logs/$useless_outputs_logfile_name #Remove previous scalations (closing outputs)
		rm logs/$rescale_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
		
		echo "    [i] Rescaling the estimated pointclouds using the map points..."
	
		#Scale the dense maps
		./programs/3_scalation/build/3_rescalerCOLOR_ransac $source_directory $keyFrame_file $mapPoints_file $depthEstimation_directory $depthEstimation_filename_root $depthEstimation_FULL_filename_root $depthEstimation_filename_extension $scalation_directory $pointcloud_filename_root $pointclouds_filename_extension &>logs/$rescale_logfile_name #2.5 min aprox (62 nubes)
	fi	
else
	echo "[i] Omitted, the user just wants to visualize data"
fi






#################################
####  R E F I N E M E N T S  ####
#################################

echo "

REFINE THE OBTAINED DENSE MAP
============================="

#Set output directory
def_refinement_directory="output_files/4_refinement/"
refinement_directory=$def_refinement_directory

#Set output filename's root
refinement_filename="fine_map.pcd_"

#Set logfile name
refinement_logfile_name=4_refinement.log

if test $just_visualize == "no"
then
	#Ask if the user wants to use already estimated depths
	read -p "[>] Would you like to use an already refined map? (yes/no): " use_refined_map
	
	#Localize estimated depths (or generate them)
	if test $use_refined_map == "yes"
	then
		#Get path to source directory
		echo    "    [i] Great! We will work with a refined map!"
		read -p "    [>] Please, introduce the refined map's global path (default: $def_refinement_directory): " refinement_directory
	
		#Check if the user wanted the default directory (he did not entered any input)
		if test "$refinement_directory" != ""
		then
			#Remove old refinements
			rm $def_refinement_directory* &>logs/$useless_outputs_logfile_name #Remove previous refinement (closing outputs)
			rm logs/$refinement_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
	
			#Copy the reprojections from the given folder to this workspace
			cp $refinement_directory$refinement_filename $def_refinement_directory 
		else
			refinement_directory=$def_refinement_directory 
			echo "        [i] No input detected! Using '$refinement_directory' as the refined map directory"
		fi
	else
		#Remove old refinements
		rm $def_refinement_directory* &>logs/$useless_outputs_logfile_name #Remove previous refinement (closing outputs)
		rm logs/$refinement_logfile_name &>logs/$useless_outputs_logfile_name #Remove previous logfile (closing outputs)
		
		echo "    [i] Refining the rough dense maps..."
	
		#Refine the maps
		./programs/4_refination/build/4_refinator $source_directory $mapPoints_file $scalation_directory $pointcloud_filename_root $pointclouds_filename_extension &>logs/$refinement_logfile_name
	fi	
else
	echo "[i] Omitted, the user just wants to visualize data"
fi






#######################################
####  V I S U A L I Z A T I O N S  #### 
#######################################

echo "

VISUALIZE THE RESULTS
====================="


#Set output directories
#def_visual_kpsReproj_directory="output_files/4_visualizationAndTesting/reprojection_visualization"
#visual_kpsReproj_directory=$def_scalation_directory
#
##Set output filenames's roots
#visual_kpsReproj_filename_root="kps_reproj_"
#
##Set output filenames's extensions
#visual_kpsReproj_filename_extension=".png"

#Set logfiles names
visual_kpsReproj_logfile_name=5_visual_kpsReproj.log
visual_denseMap_logfile_name=5_visual_denseMap.log
visual_sparseMap_logfile_name=5_visual_sparseMap.log
visual_fine_logfile_name=5_visual_fine.log
visual_demon_logfile_name=5_visual_demon.log


visualization_desire="foo"

while test "$visualization_desire" != "none" &&  test "$visualization_desire" != "n" &&  test "$visualization_desire" != "q"
do
	#Ask if the user wants to visualize some results
	read -p "[>] What results would you like to visualize? (none|n|q / reprojection|r / map|m): " visualization_desire
	
	case $visualization_desire in
		none|n|q)
			echo "    [i] OK! I will clean everything and exit the script!"
			;;
		reprojection|r)
			#TODO - Allow the user to save whatever visualization he wants to
			#TODO - Allow the user to go to the previous image
			echo "    [i] Great! I will show you the keypoints of each keyframe alongside their reprojections!"
			echo "    [i] Double-tap 'n' to see next frame or 'q' to quit"
	
			python programs/5_visualization/reprojection/visualizeReprojection.py $images_directory/rgb/ $source_directory $keyFrame_file $keyPoints_file $reprojection_directory $reprojectedKeyPoints_filename_root $reprojectedKeyPoints_filename_extension &>logs/$visual_kpsReproj_logfile_name
			;;
		map|m)
			#TODO - Allow the user to merge and save the pointcloud
			#Ask if the user wants to visualize just the sparse map or both the sparse and the dense one
			read -p "    [>] What map would you like to visualize? (dense/sparse/fine/demon): " map_desire
			if test "$map_desire" == "dense"
			then
				echo "        [i] Great! I will show you the resulting dense map alongside the original sparse one (green)!"
				echo "        [i] Tap 'q' to quit. It might take a while to load every pointcloud, please wait..."

				./programs/5_visualization/maps/build/visualize_denseMap $source_directory $mapPoints_file $scalation_directory $pointcloud_filename_root $pointclouds_filename_extension &>logs/$visual_denseMap_logfile_name
			elif test "$map_desire" == "fine"
			then
				echo "        [i] Great! I will show you the fine map!"

				./programs/5_visualization/maps/build/visualize_fineMap &>logs/$visual_fine_logfile_name
			elif test "$map_desire" == "demon"
			then
				echo "        [i] Great! I will show you the demon (sparse) maps!"

				./programs/5_visualization/maps/build/visualize_demon $source_directory $keyFrame_file $mapPoints_file $depthEstimation_directory $depthEstimation_filename_root  $depthEstimation_filename_extension &>logs/$visual_demon_logfile_name
			else
				echo "        [i] Great! I will show you the sparse map in red (alongside the followed trajectory, in green)!"
				echo "        [i] Tap 'q' to quit"

				./programs/5_visualization/maps/build/visualize_sparseMap $source_directory $mapPoints_file $keyFrame_file &>logs/$visual_sparseMap_logfile_name
			fi
			;; 
		*)
			echo "    [!] ERROR! The introduced option was not valid. Valid options: 'nothing', 'visualizeReprojection' or 'visualizeDenseMap'"
			;;
	esac
done

