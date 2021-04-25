# ENPM661 : Project3 - Phase 3 - Part 1
## Introduction
* In this project a 10x10 grid is provided where there are five obstacles present. A rigid differential drive robot with non holonomic contraints is given a random start and goal position by the user, and it is required to find its path to the goal location.
* A* search method is used in this project for searching the goal location which takes into consideration the cost to come as well as cost to go such that it satiesfies the differential and non holonomic constraints of the robot. The exploration is shown by visualization using half planes method and also the final optimal path is represented through visualization.
## Files
* There are three files in the folder. One is the A* python code, one is the video recording and one is the README file. The name of python file is as follows:
* Phase_3.py
## Libraries Required
* OpenCv
* Numpy
* sys
* datetime
* math
-----------------------------------------------------------------------
# Instructions For Running the Phase 1 code:
* Open the python file in Spyder/Pycharm IDE. 
* Check whether all the libraries are installed or not.
* NOTE: Tester MUST go into the code and manually key in the following inputs for the test.....
	a.START points: variable name TestCaseXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 52)
	b. END points: variable name FinalStateXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 60)
	c. Robot RADIUS  (line 84)
	d. Robot CLEARANCE (line 85)
	e. THETA direction  (line 86)
	f. Robot STEP size(Not Above 10)  (line 87)
* Run the code.
* Note: An output NodePath.txt file with some test case results will overwrite every time.
	a. In the output NodePath.txt file, "Our road map" is the parent-child node map from initial to goal state. 
* Note: Nodes are stored as pixel coordinates, not standard x-y coordinates (only difference is the Y pixel coordinate will be inverted) 	 
* A visualization graph as well as optimal path will be shown if valid start and goal location co-ordinates are provided. And a video file will be saved in the folder.The output video is called "Sweeping_Phase_3.mp4".
