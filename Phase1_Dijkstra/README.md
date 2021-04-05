# ENPM 661-Project 3 Phase 1
## Introduction
* In this project a 300x400 grid is provided where there are four obstacles present. A rigid robot is given a random start and goal position by the user, and it is required to find its path to the goal location such that it avoids collision with the obstacles as well as maintain the required clearance.
* Dijkstra algorithm is used in this project which takes into consideration the cost to come for searching the goal location and the exploration is shown by visualization using half planes method. Moreover, the final optimal path is also represented through visualization.
## Files
* There are three files in the folder. One is the Dijkstra python code, one is the video recording and one is the README file. The name of python file is as follows:
* Phase_1.py
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
* NOTE: Tester MUST go into the code and manually key in the start and end points for the test. 
	a. START points: variable name TestCaseXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 25)
	b. END points: variable name FinalStateXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 39)
 * Run the code.
 * Note: An output NodePath.txt file with some test case results will overwrite every time.
	a. In the output NodePath.txt file, "Our road map" is the parent-child node map from initial to goal state. 
* Note: Nodes are stored as pixel coordinates, not standard x-y coordinates (only difference is the Y pixel coordinate will be inverted) 	 
* A visualization graph as well as optimal path will be shown if valid start and goal location co-ordinates are provided. And a video file will be saved in the folder.The output video is called "Sweeping_Phase_1.mp4".
