Project 5:

Tester inputs: 

1.  Heading angle set by Theta_Start = 0 on line 45 of the Project_5.py script. 
    It is recommended to keep it this way if you want to see the ROS output, because
    the turtlebot spawns at a heading of 0 degrees as well.
2.  Initial X,Y coordinate set by TestCaseCOOR = [8, 8] on line 46 of the Project_5.py script
3.  Goal X,Y coordinate set by FinalStateCOOR = [9, 9] on line 47 of the Project_5.py script
4.  Set plotting_ROS_too = True (on line 15) to see the ROS simulation. 
5.  Set Fast_RRT = True (on line 16) of the Project_5.py script to see the Fast_RRT method implemented.
    False indicates Basic RRT.  

Dependencies: 
numpy , cv2 , sys , datetime , math , time , random, warnings, rospy , geometry_msgs.msg 

To run without ROS output:
1. If running with Spyder, open the Project_5.py script from Spyder IDE and click the run icon. 
2. If running with Linux, open a terminal and type 'python3 Project_5.py' in the scripts directory. 
 
To run with ROS output: 
1.  Remember to set the flags for plotting_ROS_too = True (on line 15)
2.  The node package name is 'project_5'. Please try to keep this the same when building the catkin package,
    as the launch file is configured for this node package name as well. 
3.  From the Linux terminal window type: roslaunch project_5 project_5_turtlebot.launch 
4.  To test a different set of initial/goal points, remember to update both the Project_5.py script lines 46-47
    and the launch file with the proper spawn location. 


Directory structure:

project_5   <---(node package name)
   |
   launch
   |    |
   |    project_5_turtlebot.launch
   world
   |   | 
   |   proj5map.world
   scripts
   |	 |
   |	 Project_5.py
   src
   | |
   |  (empty)
   |
   CMakeLists.txt
   |
   package.xml 
   |
   README.md
   |
   ROSoutput_Basic_RRT_1559 (showing test case (1,5) to (5,9) with Basic RRT)
   |
   ROSoutput_Fast_RRT_1559 (showing test case (1,5) to (5,9) with Fast RRT)  


Code Output: 
The terminal window will output the roadmap from the start to goal nodes and the time to find the goal. 
In addition, it will output the Velocity map if the ROS simulation is selected. The velocity map is a
map of (linear velocity, angular velocity) tuples that correspond to each node in the roadmap. 
These values are fed to turtlebot to execute the roadmap ROS visualization. 

In the scripts directory, Sweeping_Basic_RRT.mp4 will show the output visualization when Fast_RRT=False.
Sweeping_Fast_RRT.mp4 will show the output visualization when Fast_RRT=True. 