# Adaptive Human-Robot motion transfer for complete body imitation
What does this repository do?

When properly setup you can use your body motions to control a virtual NAO robot using rviz. Move your arm the virtual NAO will move its hand. Move your leg the NAO will mimic.


This repository uses ROS, Rviz, the Kinect V2, and virtual NAO robot. 


The user must install ROS at a Kinetic or more recent version. This was setup for Kinetic.

     http://wiki.ros.org/Installation/Ubuntu

The user also must install catkin to build and run this project.

    http://wiki.ros.org/catkin


I used https://github.com/msr-peng/kinect_v2_skeleton_tracking repository to link kinect_V2 from windows machine to an ubuntu machine. At the time of creating this repository there wasn’t much support with ROS on windows nor was there any real ubuntu port of Microsoft Kinect_V2. Thus, msr-peng repository was used as one of the foundations of this repository.
The idea is the following

 ![image](https://user-images.githubusercontent.com/79240616/166574980-d23051e3-0714-4c78-949c-cde542ce2e77.png)

	

 https://rc.library.uta.edu/uta-ir/bitstream/handle/10106/30244/VILLA-THESIS-2021.pdf?sequence=1&isAllowed=y
