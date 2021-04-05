# 661-Project 3 Phase 1 & 2

The script names are Phase_1.py (Dijkstra rigid robot) and Phase_2.py (A star with vectors)
The packages imported in the code are numpy, sys, cv2, math, and datetime 
-----------------------------------------------------------------------
Running the Phase 1 code:
1A. To run in Spyder IDE ---execute the script with the "Run" icon. 
1B. To run in XYZ --- Dhyey if you want to put in how to run in pycharm feel free

NOTE: Tester MUST go into the code and manually key in the start and end points for the test. 
	a. START points: variable name TestCaseXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 25)
	b. END points: variable name FinalStateXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 39)


Running the Phase 2 code:
1A. To run in Spyder IDE ---execute the script with the "Run" icon.
1B. To run in XYZ --- Dhyey if you want to put in how to run in pycharm feel free

NOTE: Tester MUST go into the code and manually key in the following inputs for the test.....
	a.START points: variable name TestCaseXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 32)
	b. END points: variable name FinalStateXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map (line 45)
	c. Robot RADIUS  (line 84)
	d. Robot CLEARANCE (line 85)
	e. THETA direction  (line 86)
	f. Robot STEP size  (line 87)

--------------------------------------------------------------------------
Note: An output NodePath.txt file with some test case results will overwrite every time.
	a. In the output NodePath.txt file, "Our road map" is the parent-child node map from initial to goal state. 
	
Note: Nodes are stored as pixel coordinates, not standard x-y coordinates (only difference is the Y pixel coordinate will be inverted) 	 

Code Output: The output video is called "Sweeping.mp4"
