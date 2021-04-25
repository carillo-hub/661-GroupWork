Tester inputs: 
Heading angle set by Theta_Start = 0 on line 51. It is recommended to keep it this way if you want to see the ROS output, because
the turtlebot spawns at a heading of 0 degrees as well. If only wanting to see the output visualization in OpenCV, any heading 
angle is fine. 
Initial X,Y coordinate set by TestCaseCOOR = [8, 8] on line 52
Goal X,Y coordinate set by FinalStateCOOR = [9, 9] on line 60
Set plotting_ROS_too = True &  plotting_Viz_only = False (on line 12 & 13) if a ROS output is desired 
Set plotting_Viz_only = True & plotting_ROS_too = False (on line 12 & 13) if only a standard OpenCV visualization is desired

Configuration:
The threshold to reach the goal is chosen to be 15 pixels. RPM1 is 5 and RPM2 is 10. 

Dependencies: 
numpy , cv2 , sys , datetime , math , time , rospy , geometry_msgs.msg 

To run with ROS output: 
Remember to set the flags for plotting_ROS_too = True &  plotting_Viz_only = False (on line 12 & 13)
From the Linux terminal window type: roslaunch phase3 turtlebot3_map_world.launch 
To test a different set of initial/goal points, remember to update the launch file with the proper spawn location


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


Code Output: 
The terminal window will output the roadmap from the start to goal nodes. In addition, it will output the Velocity map. The
velocity map is a map of (linear velocity, angular velocity) tuples that correspond to each node in the roadmap. These values are
fed to turtlebot to execute the roadmap ROS visualization. 