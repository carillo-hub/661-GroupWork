# ENPM661 : Project3 - Phase 3 - Part 1
## Introduction
* In this project a 10x10 grid is provided where there are five obstacles present. A rigid differential drive robot with non holonomic contraints is given a random start and goal position by the user, and it is required to find its path to the goal location.
* A* search method is used in this project for searching the goal location which takes into consideration the cost to come as well as cost to go such that it satiesfies the differential and non holonomic constraints of the robot. The exploration is shown by visualization using half planes method and also the final optimal path is represented through visualization.
## Files
* There are four files in the folder. One is the A* python code, there are two video recordings for test case [6,8] to [9,9] and [8,5] to [7,7] respectively.And one is the README file. The name of python file is as follows:
* Phase_3.py
## Libraries Required
* OpenCv
* Numpy
* sys
* datetime
* math
* time
-----------------------------------------------------------------------
# Instructions For Running the Phase 1 code:
* Open the python file in Spyder/Pycharm IDE. 
* Check whether all the libraries are installed or not.
* Set plotting_Viz_only = True & plotting_ROS_too = False (on line 12 & 13) if only a standard OpenCV visualization is desired (part 1).
* NOTE: Tester MUST go into the code and manually key in the following inputs for the test.....
* a. START points: variable name TestCaseXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 52). Default start point is [6,8].
* b. END points: variable name FinalStateXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 60). Default goal point is [9,9].
* c. Enter Two separate robot RPM values  (line 58 & line 59). The default values are RPM1 = 5, RPM2 = 10. 
* d. Initial HEADING angle: variable name Theta_Start (line 51). The default value is 0 degrees. 
* Run the code.
* Note: The terminal window will output the node roadmap for the optimal path from start to goal. In addition, the Velocity map of tuple linear and angular velocities for each node in the roadmap is also printed to the terminal. 
* Note: Nodes are stored as pixel coordinates, not standard x-y coordinates (only difference is the Y pixel coordinate will be inverted) 	 
* A visualization graph as well as optimal path will be shown if valid start and goal location co-ordinates are provided. And a video file will be saved in the folder.The output video is called "Sweeping.mp4".

