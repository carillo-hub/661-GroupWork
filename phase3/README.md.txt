Project 3 Phase 3 Part 2 ROS Simulation:

Tester inputs: 

1.  Heading angle set by Theta_Start = 0 on line 51 of the Phase_3.py script. 
    It is recommended to keep it this way if you want to see the ROS output, because
    the turtlebot spawns at a heading of 0 degrees as well.
2.  Initial X,Y coordinate set by TestCaseCOOR = [8, 8] on line 52 of the Phase_3.py script
3.  Goal X,Y coordinate set by FinalStateCOOR = [9, 9] on line 60 of the Phase_3.py script
4.  Set plotting_ROS_too = True &  plotting_Viz_only = False (on line 12 & 13) to see the ROS simulation.  

Configuration:
The threshold to reach the goal is chosen to be 15 pixels. RPM1 is 5 and RPM2 is 10. 

Dependencies: 
numpy , cv2 , sys , datetime , math , time , rospy , geometry_msgs.msg 

To run with ROS output: 
Remember to set the flags for plotting_ROS_too = True &  plotting_Viz_only = False (on line 12 & 13)
From the Linux terminal window type: roslaunch phase3 turtlebot3_map_world.launch 
To test a different set of initial/goal points, remember to update both the Phase_3.py script and
the launch file with the proper spawn location. 


Directory structure:

phase3
   |
   launch
   |    |
   |    turtlebot3_map_world.launch
   world
   |   | 
   |   map.world
   scripts
   |	 |
   |	 Phase_3.py
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
   ROSoutput_6899 (showing test case 1 simulation)
   |
   ROSoutput_8577 (showing test case 2 simulation)   



Code Output: 
The terminal window will output the roadmap from the start to goal nodes. In addition, it will output the Velocity map. The
velocity map is a map of (linear velocity, angular velocity) tuples that correspond to each node in the roadmap. These values are
fed to turtlebot to execute the roadmap ROS visualization. 