# 661-Project 3 Phase 1 & 2

The script names are Phase_1.py and Phase_2.py
The packages imported in the code are numpy, sys, cv2, and datetime *dont remember right now if this is all of them
-----------------------------------------------------------------------
Running the code:
1A. To run in Spyder IDE ---execute the script with the "Run" icon. 
	a. Tester MUST go into the code and manually key in the start and end points for the test. Dhyey note: i think we need to include instructions for keying in the radius/clearance/step/theta right? 
	b. START points: variable name TestCaseXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map
	c. END points: variable name FinalStateXY = [x,y], where x and y are the standard orientation coordinates from the obstacle map
	d. TestCaseXY variable is defined on line YZ of the code, FinalStateXY is defined on line XYZ.

1B. To run in XYZ --- Dhyey if you want to put in how to run in pycharm feel free

2. The script will run only one testcase at a time, and the NodePath.txt file with results will overwrite every time.

3. As a note, in the output NodePath.txt file, "Our road map" is the parent-child node map from initial to goal state. 
Nodes are stored as pixel coordinates, not standard x-y coordinates (only difference is the Y pixel coordinate will be inverted) 	 

4. The output video is called "Sweeping.mp4"
