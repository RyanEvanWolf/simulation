# simulation
vSLAM simulator over a small window in a ROS based framework. Integrated into vSLAM_Analysis

To run the simulation generator use:
rosrun simulation generateMotion

-----------------------
This will create a set of pickled file objects that contain the pose vertices for an ideal case (.simulation) and gaussian injected noise case (.gauss).
Note that it expects the folder structure to be present, and the number of tracks has to be manually adjusted in the python script at line 32.




To extraction motion using the RANSAC method use:
rosrun simulation extractMotion

-----------------------------------
This will populate the motion edges of the pose graph at the specified rootFolder.
This folder entry maust be manually changed at line 19



------------------------
