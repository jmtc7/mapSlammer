# Map Slammer: Densifying Scattered KSLAM 3D Maps with Estimated Depth

This repository contains the source files and some other related documents regarding my end-of-degree thesis. This work has been supported by the Spanish Government TIN2016-76515R Grant, supported with European Regional Development Fund (ERDF) funds. I have been supported by a Spanish Government grant for cooperating with the [RoVit research group](http://www.rovit.ua.es/) in research tasks ID 998142. This work has been submitted to the [ROBOT’19](https://web.fe.up.pt/~robot2019/index.php) conference, an international congress that will be held in O Porto, Portugal, in November 2019.

A demo video of its performance is available on YouTube:

[![Demo video](docs/demo.gif)](https://www.youtube.com/watch?v=b74P3ykYE34)


## Approach

This project consists in a method to create a dense three-dimensional representations of the environment in which a single 2D camera is moving with an unconstrained movenet. This is done by fusing the output of a keyframe-based visual SLAM algorithm ([ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)) with predicted depthmaps obtained with a Deep Learning method ([DeMoN](https://github.com/lmb-freiburg/demon)).

The current results provide maps that densify the ORB-SLAM2 output extracting almost 200 times more data from each keyframe (49152 vs 250.54 3D points). The resulting map's average distance to the nearest neighbours of the groundtruths is 0.0399 meters, with a variance of 0.0006. This measures have been computed using various sequences from the [TUM RGB-D SLAM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset).


## File structure

The files in this repository are divided in the following parts (files/directories):
* **docs**: Directory containing the documentation files and media. It contains my [Degree Thesis](https://github.com/jmtc7/mapSlammer/blob/master/docs/MapSlammer_JoseMiguel_TorresCamara_DegreeThesis.pdf), which can be also found in the institutional Repository of the University of Alicante ([RUA](https://rua.ua.es/dspace/bitstream/10045/94751/1/SLAM_usando_tecnicas_de_deep_learning_Torres_Camara_Jose_Miguel.pdf)). The paper itself can be requested from [ResearchGate](https://www.researchgate.net/publication/337400838_Map_Slammer_Densifying_Scattered_KSLAM_3D_Maps_with_Estimated_Depth) or found in this [Springer book](https://doi.org/10.1007/978-3-030-36150-1_46).
* **programs**: Directory containing the source files and the corresponding CMakeLists.
* **full_runCOLOR.bash**: Bashscript integration of the whole project to ease and automate the exercution. It incorporates command-line based interaction with the user that allows some personalization of the execution


## Instalation and usage

TODO



# TODOs

Repository tasks:
* Usage tutorial.

Technical tasks:
* Generate one unique CMakeList.txt to compile the whole project. 
* For each optimisation idea or new feature I come up with, I write a comment in its correspondant source file starting by "TODO".
