import cv2
import os
import sys

if len(sys.argv) != 8:
    print("[!] ERROR! The number of expected arguments was 8")
    print("  [!] Usage: python visualizeReprojection.py images_directory ORBSLAM2_output_directory keyFrame_filename keyPoints_filename  reprojection_output_directory reprojections_filename_root reprojection_filename_extension") 
    sys.exit()

#Parse arguments
images_path = sys.argv[1]
orbSlam_output_path = sys.argv[2]
keyFrameTrajectory_filename = sys.argv[3]
keyPoints_filename = sys.argv[4]
reprojection_path = sys.argv[5]
reprojection_nameRoot = sys.argv[6]
reprojection_fileExtension = sys.argv[7]




#Iterate over the lines in the file used as the list of frames
image_list_file = open(orbSlam_output_path+keyFrameTrajectory_filename, "r")
for line in image_list_file:
    #Get image name (and parse the rest of useless datand parse the rest of useless dataa)
    current_frame, x_3d, y_3d, z_3d, q1, q2, q3, q4 = line.split()
    
    #Read image
    #TODO - Parametrize images extension
    img = cv2.imread(images_path + current_frame + ".png")

    #Read real keypoints (extracted by ORB SLAM 2)
    real_kps_file = open(orbSlam_output_path+keyPoints_filename, "r")
    real_kps = [] #List of real keypoints (in the current frame)
    for line in real_kps_file:
        timestamp, x, y = line.split()
        if timestamp == current_frame:
            real_kps.append([float(x), float(y)])
    real_kps_file.close()

    reprojected_keypoints_filename = reprojection_path + reprojection_nameRoot + current_frame + reprojection_fileExtension
    #Read reprojected keypoints (using the 3D map)
    if os.path.isfile(reprojected_keypoints_filename): #If there are reprojected keypoints for this frame (this condition is only useful if the process has not been executed over every keyframe)
        reproj_kps_file = open(reprojected_keypoints_filename, "r")
        reproj_kps = [] #List of reprojected keypoints (in the current frame)
        for line in reproj_kps_file:
            x, y = line.split()
            
            
            #Append to the list of reprojected keypoints
            reproj_kps.append([float(x), float(y)])
        reproj_kps_file.close()
    
        #Print both the real keypoints (groundtruth, in green and bigger)
        for kp in real_kps:
            cv2.circle(img, (int(kp[0]),int(kp[1])), 3, (0,255,0), -1)

        #Print the reprojected keypoints (in red and smaller)
        for kp in reproj_kps:
            cv2.circle(img, (int(kp[0]),int(kp[1])), 2, (0,0,255), -1)
    
        #Show image
        cv2.imshow("Keypoints viewer", img)
        
        #Go to the next image or close
        #TODO - Add an option 's' to save current visualization
        if cv2.waitKey(0) & 0xFF == ord('n'):
            print ("[*] Showing next image's keypoints...")
        if cv2.waitKey(0) & 0xFF == ord('q'):
            print ("[*] Shutting everything down...")
            break

image_list_file.close()
