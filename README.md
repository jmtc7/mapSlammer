# Map Slammer: Densifying Scattered KSLAM 3D Maps with Estimated Depth

This repository contains the source files and some other related documents regarding my end-of-degree thesis. This work has been supported by the Spanish Government TIN2016-76515R Grant, supported with European Regional Development Fund (ERDF) funds. I have been supported by a Spanish Government grant for cooperating with the [RoVit research group](http://www.rovit.ua.es/) in research tasks ID 998142. This work has been submitted to the [ROBOT’19](https://web.fe.up.pt/~robot2019/index.php) conference, an international congress that will be held in O Porto, Portugal, in November 2019.


# Approach

This project consists in a method to create a dense three-dimensional representations of the environment in which a single 2D camera is moving with an unconstrained movenet. This is done by fusing the output of a keyframe-based visual SLAM algorithm ([ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)) with predicted depthmaps obtained with a Deep Learning method ([DeMoN](https://github.com/lmb-freiburg/demon)).

The current results provide maps that densify the ORB-SLAM2 output extracting almost 200 times more data from each keyframe (49152 vs 250.54 3D points). The resulting map's average distance to the nearest neighbours of the groundtruths is 0.0399, with a variance of 0.0006. This measures have been computed using various sequences from the [TUM RGB-D SLAM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset).


# File system

The files in this repository are divided in the following parts (files/directories):
* **programs**: Directory containing the source files and the corresponding CMakeLists.
* **full_runCOLOR.bash**: Bashscript integrating the whole project. It incorporates command-line based interaction with the user to perform a custom (or not) run.
* **docs**: Directory containing the documentation files (pending of upload).


# Instalation and usage

TODO



# TODOs

Repository tasks:
* Add result image
* Add documentation (end-of-degree thesis report and/or paper) -> HRef.
* Usage tutorial.

Technical tasks:
* Generate one unique CMakeList.txt to compile the whole project. 
* For each optimisation idea or new feature I come up with, I write a comment in its correspondant source file starting by "TODO".
