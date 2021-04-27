#!/usr/bin/python3
import numpy as np
import cv2
import sys
import datetime
import math
import time
import random

starttime = datetime.datetime.now()
print("start time: ", starttime)

plotting_Viz_only = True  #true if do not want to use ROS
plotting_ROS_too = False  #true if want to use ROS

if plotting_ROS_too == True:  #give ROS time to load up 
    time.sleep(20)
    import rospy
    from geometry_msgs.msg import Twist
    
#############################################################################################
# Initial / Final State Info  ********TESTER INPUT NEEDED HERE FOR START/GOAL POINTS**********
#############################################################################################

# Initializing some values
global ctr
global closed_list
global start_node_cpy
global FinalStateID
global yscale
global xscale
global radius
global clearance
global threshold
global total_clearance
global scaling
scaling = 100              #makes map a 1000x1000
yscale = 10 * scaling
xscale = 10 * scaling
closed_list = []
ctr = 0
VisitedDict = {}
VelDict = {}
radius = 0.038  * scaling  # robot radius in pixels taken from the data sheet 5
L = 0.354  *scaling        #wheel distance L taken from the data sheet 2
threshold = 0.5            # *** THIS IS NOT CONFIGURABLE. DO NOT CHANGE
total_clearance = 0.125 * scaling # gives a clearance of 12.5 m in the scaled space



# Get req'd inputs from tester
Theta_Start = 0  # <--------------------------TESTER PUT INITIAL HEADING ANGLE IN DEGREES HERE
TestCaseCOOR = [3,6] # <---------------------TESTER PUT INITIAL X,Y COORDINATE PT HERE
RPM1 = 10   
RPM2 = 20  
FinalStateCOOR = [9,9 ] # <-------------------TESTER PUT GOAL X,Y COORDINATE PT HERE
 

#apply scaling to coors
initX = TestCaseCOOR[0] * scaling 
initY = TestCaseCOOR[1] * scaling
TestCaseXY = [initX,initY, Theta_Start]  
print("\nInitial State node cartesian (x,y) ", TestCaseXY)
finX = FinalStateCOOR[0] * scaling 
finY = FinalStateCOOR[1] * scaling
FinalStateXY = [finX, finY] 
print("Final state node cartesian (x,y): ", FinalStateXY)  


#translate cartesian (col,row) to pixel coors (row,col)
y = yscale - TestCaseXY[1] #pixel row
x = TestCaseXY[0]          #pixel column
initial_state = [y,x, Theta_Start]  # pixel coors (row,col)
start_node_cpy = [y,x, Theta_Start]  # used for plotting the backtracking later
y = yscale - FinalStateXY[1]
x = FinalStateXY[0]
FinalState = [y,x]  # row,col pixel coordinates
FinalStateID = tuple(FinalState)  # pixel coors


#############################################################################################
# Video writer info
#############################################################################################

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 50
outputVideo = cv2.VideoWriter("Sweeping.mp4", fourcc, fps, (xscale, yscale))  # output resolution must match input frame (60% resized from 1980x1020)
print("\n...Creating Video...\n")


#############################################################################################
# Plot -- READS PIXEL COORS and plots pixel coors
#############################################################################################

def space(CurrentNode, mask, color, thickness):
    global FinalState  # PIXEL COORS
    global initial_state  # pixel coors

    # Always draw in Obstacle shapes
    cv2.circle(mask, (2*scaling, (yscale - 2*scaling)), 1*scaling, (128, 255, 128), 2)
    cv2.circle(mask, (2*scaling, (yscale - 8*scaling)), 1*scaling, (128, 255, 0), 2)
    for i in range(yscale):
        for j in range(xscale):
            y_d = i
            y_d = yscale - y_d
            x_d = j
            if ((3.75* scaling) <= x_d <= (6.25 * scaling) and (4.25* scaling) <= y_d <= (5.75* scaling)) or ((7.25* scaling) <= x_d <= (8.75* scaling) and (2* scaling) <= y_d <= (4* scaling)) or  ((0.25* scaling) <= x_d <= (1.75* scaling) and (4.25* scaling) <= y_d <= (5.75* scaling)):
                mask[i : i + 1 , j : j +1] = (128, 255, 0)


    # Always draw in Initial/Final State points
    cv2.circle(mask, (int(FinalState[1]), int(FinalState[0])), 1, (255, 0, 0), 5)
    cv2.circle(mask, (int(initial_state[1]), int(initial_state[0])), 1, (255, 0, 0), 5)

    # Draw in Current Node points -- NEEDS PIXEL COORS
    cv2.circle(mask, (int(CurrentNode[1]), int(CurrentNode[0])), 1, color, thickness)

    return mask

# Initialize the plot in the video frame sequence
global mask
mask = 255 * np.ones((1000 , 1000 , 3), np.uint8)  # mask to plot on
mask = space(initial_state, mask, (0, 0, 0),5)  # initialize the plot
outputVideo.write(mask)  # output frame to video sequence


#############################################################################################
# Queue method
#############################################################################################
class Queue:

    def __init__(Visited):
        Visited.items = []  # This is our Visited list

    def enqueue(Visited, item):  # Adding items to Visited
        Visited.items.insert(0, item)

    def size(Visited):  # Determine length of Visited
        return len(Visited.items)

    def dequeue(Visited):  # Deleting items from Visited
        if Visited.items:
            return Visited.items.pop()
        else:
            return None


VisitedQ = Queue()  # Initialize the visited QUEUE - CLOSED (explored) NODE LIST
VisitedQ_eachRound = []  # Initialize the visited LIST - OPEN (not yet explored) NODE LIST


#############################################################################################
# Define boundaries and obstacles (reads nodes as XY coods, checks if in obstacles PIXEL coordinates)
#############################################################################################


def out_of_bounds(Node):
    global xscale
    global yscale
    x = Node[1]
    y = Node[0]

    if x > (xscale - total_clearance) or y > (yscale - total_clearance) or x < 0 + total_clearance or y < 0 + total_clearance:
        return True
    else:
        return False
def in_circle(Node):
    global xscale
    global yscale
    x = Node[1]
    y = Node[0]
    r = 1 * scaling

    # if inside circle, return true
    if (x - (2 * scaling)) ** 2 + (y - (yscale - (2 * scaling))) ** 2 <= (r + total_clearance) ** 2:
        return True
    elif (x - (2 * scaling)) ** 2 + (y - (yscale - (8 * scaling))) ** 2 <= (r + total_clearance) ** 2:
        return True
    else:
        return False
def in_rect_new(Node):
    global xscale
    global yscale
    x = Node[1]
    y = Node[0]
    #y = yscale - y

    if x >= ((3.75* scaling) - total_clearance) and x <= ((6.25 * scaling) + total_clearance) and y <= ((5.75* scaling) + total_clearance) and y >= ((4.25* scaling) - total_clearance):
        return True
    elif x >= ((7.25* scaling) - total_clearance) and x <= ((8.75* scaling) + total_clearance) and y <= ((4* scaling) + total_clearance) and y >= ((2* scaling) - total_clearance):
        return True
    elif x >= ((0.25* scaling) - total_clearance) and x <= ((1.75* scaling) + total_clearance) and y <= ((5.75* scaling) + total_clearance) and y >= ((4.25* scaling) - total_clearance):
        return True
    else: return False
def in_obstacles(Node):
    in_rect_nw = in_rect_new(Node)
    in_circ = in_circle(Node)
    out_of_bound = out_of_bounds(Node)

    if in_rect_nw == True or in_circ == True or out_of_bound == True:
        return True
    else:
        return False


#############################################################################################
# Verify Initial/Goal state are in boundaries and out of obstacles, Initialize the Plot
#############################################################################################

# TRUE = failed obstacle check
obstacles_check_GOAL = in_obstacles(FinalState)
obstacles_check_INITIAL = in_obstacles(initial_state)

if obstacles_check_GOAL == True:  # impose constraint
    print("\nfinal state IN an obstacle -- choose another please\n")
    sys.exit()

if obstacles_check_INITIAL == True:  # impose constraint
    print("\ninitial state IN an obstacle -- choose another please\n")
    sys.exit()


#############################################################################################
# Plot the Optimal Path -- PLOTTING IN PIXEL COORS
#############################################################################################

def Talker(OurMap,OurVelMap):
    msg = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('Talker',anonymous=True)
    
    for o in range(0,len(OurVelMap)):
    	
        msg.linear.x = OurVelMap[o][0]  /scaling 
        msg.angular.z = OurVelMap[o][1]  * -1
        pub.publish(msg)
        time.sleep(1.5)

    
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
    
    print("Our NODE Map = ",OurMap)
    print("\nOur VELOCITY Map = ",OurVelMap)
    print("\n\n Number of steps in Road Map is ",len(OurMap))
        
    
    # Video compelte, calculate code run time
    endtime = datetime.datetime.now()
    runtime = endtime - starttime
    print("\nRun time: ", runtime)
    print("\nDone. Can see results in Sweeping.mp4!")
    sys.exit()

def Plot_OurMap(OurMap,OurVelMap):
    global mask
    for EachID in OurMap:
        x = int(EachID[0])
        y = int(EachID[1])
        CurrentNode = [x, y]

        mask = space(CurrentNode, mask, (255, 0, 0), 5)
        outputVideo.write(mask)
        cv2.imshow("Plotting", mask)
        cv2.waitKey(1)
            
    # Video compelte, calculate code run time
    outputVideo.release()
    cv2.destroyAllWindows()
    

    if plotting_Viz_only == True:
        print("Our NODE Map = ",OurMap)
        print("\nOur VELOCITY Map = ",OurVelMap)
        print("\n\n Number of steps in Road Map is ",len(OurMap))
        endtime = datetime.datetime.now()
        runtime = endtime - starttime
        print("\nRun time: ", runtime)
        print("\nDone. Can see results in Sweeping.mp4!")
        sys.exit()


#############################################################################################
# Define parent/child roadmap for initial state to goal state --MAP IS PIXEL COORS
#############################################################################################

def roadmap(ID):
    global parent_ID
    new_vel = VelDict[ID]
    OurVelMap = []
    OurVelMap.append(new_vel)
    
    new_ID = (ID[0], ID[1], ID[2])
    OurMap = []  # initialize the roadmap
    OurMap.append(ID)  # start by adding final state tuple
    parent = VisitedDict[new_ID]  # get parent tuple
    ID = parent  # set parent -> new child

    while parent != 'Initial State':
        OurMap.append(ID)  # add child tuple to road map
        new_vel = VelDict[ID]
        OurVelMap.append(new_vel)
        parent = VisitedDict[ID]  # get parent tuple
        ID = parent  # set parent -> new child

        if parent == 'Initial State':
            OurMap.reverse()  # pitstops were grabbed in reverse order (Goal --> Initial), so need to reverse
            OurVelMap.reverse()
            break


    Plot_OurMap(OurMap,OurVelMap)
    if plotting_ROS_too == True:
        Talker(OurMap,OurVelMap)


#############################################################################################
# Define 4 functions to move the blank tile in each direction and store the NewNode in a list
#############################################################################################


def Step1(NewNode, UL, UR, ParentNode):
    global radius
    x, y = NewNode[:2]  # unpack the node into pixel x y coords
    Theta_i = NewNode[2]
    t = 0  # start integration timer
    dt = 0.1
    theta_n = 3.14 / 180 * Theta_i  #convert from degree to radian 
    x_step = x
    y_step = y
    D = 0
    ul = 2*3.14*radius/60 * UL  #convert from RPM to m/s
    ur = 2*3.14*radius/60 * UR  #convert from RPM to m/s
    xdot = radius*0.5*(ul + ur)*math.cos(theta_n)   #x velocity
    ydot = radius*0.5*(ul + ur)*math.sin(theta_n)   #y velocity
    vdot = np.sqrt((xdot**2) + (ydot**2))  #mag = linear velocity
    thetadot = radius/L*(ur-ul) #angular velocity
    velocity = (vdot,thetadot)  #send these to turtlebot

    while t < 1:
        x = x_step
        y = y_step
        t = t + dt
        x_step += 0.5 * radius * (ul + ur) * dt * math.cos(theta_n)
        y_step += 0.5 * radius * (ul + ur) * math.sin(theta_n) * dt
        theta_n += (radius / L) * (ur - ul) * dt
        D = D + math.sqrt(math.pow((0.5 * radius * (ul + ur) * math.cos(theta_n) * dt), 2) + math.pow(
            (0.5 * radius * (ul + ur) * math.sin(theta_n) * dt), 2))

        # Obstacle Check
        node = (x_step, y_step)
        flag_obc = in_obstacles(node)
        if flag_obc == True: break
        if flag_obc == False:
            cv2.arrowedLine(mask, (int(x), int(y)), (int(x_step), int(y_step)), (0, 0, 255), 1, tipLength=0.2)
            outputVideo.write(mask)
            cv2.imshow("Plotting", mask)
            cv2.waitKey(1)

        #Distance to Goal Check
        a = x_step - FinalStateID[0]
        b = y_step - FinalStateID[1]
        DistToGoal = (a ** 2 + b ** 2) ** 0.5

        if DistToGoal <= 15:  #dont want to pass the goal without seeing it
             break



    # apply the 0.5 threshold
    theta_n = 180 * (theta_n) / 3.14  #convert theta back to degree
    if theta_n > 360: theta_n = int(theta_n-360)  #check in case theta neg or too lrg
    if theta_n<0: theta_n = int(theta_n+360)
    x_raw = (2 * int((x_step)))
    x_new = x_raw / 2  
    y_raw = (2 * int((y_step)))
    y_new = y_raw / 2  

    # perform the action
    swap1 = [x_new, y_new, theta_n] 
    NewNode = swap1


def ActionSet(CurrentNode):
    NewNode = CurrentNode.copy()  # this will become the future child node
    NewNode_copy = CurrentNode.copy()  # this will become the future parent node
    actions = [(RPM1, RPM1), (RPM2, RPM2), (RPM2, 0), (0, RPM2), (RPM1, 0), (0, RPM1), (RPM1, RPM2), (RPM2, RPM1)]
    for action in actions:
        Step1(NewNode, action[0], action[1], NewNode_copy)


#############################################################################################
# Follow the flow chart
#############################################################################################

#setup initial and final state nodes into data structures 
q_0 = tuple(initial_state[:2])  # tuple pixel coors (yscaled,x) = (row,col)
VisitedDict[tuple(initial_state[:2])] = 'Initial State'
q_f = FinalStateID # tuple pixel coors (yscaled,x) = (row,col)
RRT = []  #this is the tree generated by RRT 
RRT.append(q_0)
RRT.append(q_f)
theta = Theta_Start
edges = []  #master list of all nodes in all edges
edge = np.polyfit([q_0[1], q_f[1]], [q_0[0], q_f[0]], 1)  #construct edge from start to goal nodes 
for e in range(q_0[1]+1,q_f[1]):  #take all x pts between two nodes
    f = (edge[0]  * (e)) + (edge[1])  #y=mx+b, find y 
    edge_node = (int(f),e)  #get edge node 
    edges.append(tuple(edge_node))  #add edge node to master edge node list
pt1 = (q_f[1],q_f[0])  #for plotting, pt = (col, row) 
pt2 = (q_0[1],q_0[0])
mask = cv2.line(mask, pt1, pt2, color = (127,0,127), thickness=2)
outputVideo.write(mask)
cv2.imshow("Plotting", mask)
cv2.waitKey(1)



#make Cspace dense sequnce list 
Cspace = []  #dense seq of 1000x1000 possible nodes to sample 
for n in range (0,10*scaling):
    for m in range (0,10*scaling):
        alpha = (n,m)
        Cspace.append(tuple(alpha))
Cspace.remove(q_0)   #dont include q_0 in available sample population 
Cspace.remove(q_f)   #dont include q_f in available sample population 
 
 
for i in range (0,100):
    sample = random.randint(0,len(Cspace))  #index for random sample
    alpha_i = Cspace.pop(sample)   #pops random coors from Cspace = (x,y)
    space(alpha_i, mask, (127,0,127), 8)

    
    #check vertexes of RRT tree for q nearest node
    min_dist = np.inf  
    for point in range(0,len(RRT)):    #for all nodes in RRT tree...
        a = alpha_i[0] - RRT[point][0] #find euclidean dist from each tree node to alpha_i 
        b = alpha_i[1] - RRT[point][1] 
        dist = (a ** 2 + b ** 2) ** 0.5
        if dist < min_dist:  #if this euclid dist < min euclid dist...
            min_dist = dist  #reset the min euclid dist
            q_nearest = RRT[point]  #reset the nearest node 
    VisitedDict[alpha_i] = q_nearest   #make nearest node in tree the parent of alpha_i
    
    
    #check edges of RRT tree for q nearest node 
    changed = 0  #use this to see if the q_nearest was changed after subsequent edge check 
    for points in range(0,len(edges)):  #check all edge nodes to see if there is a closer q_nearest
        c = alpha_i[0] - edges[points][0] #calc euclid dist from edge node to alpha_i 
        d = alpha_i[1] - edges[points][1]
        dist = (c ** 2 + d ** 2) ** 0.5
        if dist < min_dist: #if have a new min_dist...
            min_dist = dist   #reset the min euclid dist
            q_nearest = edges[points]  #reset the nearest node
            changed = 1   #flag to indicate an edge node was selected as q_nearest 
    if changed == 1:             #if q_nearest is an edge --> edge node becomes a vertex
        VisitedDict[alpha_i] = q_nearest  #make nearest node in tree the parent of alpha_i
        edges.remove(q_nearest)  #remove q_nearest edge node from edge list 
        RRT.append(q_nearest)    #add q_nearest as vertex to vertex list
    
    
    #add all nodes in new edge to edges list
    edge = np.polyfit([alpha_i[1], q_nearest[1]], [alpha_i[0], q_nearest[0]], 1)  #construct edge from q_nearest to alpha_i 
    if alpha_i[1]+1 < q_nearest[1]:
        low = alpha_i[1]+1 
        high = q_nearest[1]
    else: 
        high = alpha_i[1]+1 
        low = q_nearest[1]
    
    edge_open_list = []
    for e in range(low,high):  #take all x pts between two nodes 
        f = (edge[0] * (e)) + (edge[1])  #y=mx+b, find y 
        edge_node = (int(f),e)  #get edge node 
        edges.append(edge_node)  #add edge node to master edge node list 
    
    
    space(q_nearest, mask, (127,0,127), 8)    
    RRT.append(alpha_i)            #add alpha_i to RRT tree now   
    
    #draw new edge and vertex
    pt1 = (q_nearest[1],q_nearest[0])  #for plotting, pt = (col, row) 
    pt2 = (alpha_i[1],alpha_i[0])
    cv2.line(mask, pt1, pt2, color = (127,0,127), thickness=2)
    outputVideo.write(mask)
    cv2.imshow("Plotting", mask)
    cv2.waitKey(1)
        
                
        
  














