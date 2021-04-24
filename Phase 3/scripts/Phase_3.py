#!/usr/bin/python3
import numpy as np
import cv2
import sys
import datetime
import math
import time

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
global yscale
global xscale
global theta_adaptive_n
global theta_adaptive
global radius
global clearance
global theta
global step
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

# Get pixel x,y coordinates and req'd inputs from tester
Theta_Start = 0  # <------------TESTER PUT INITIAL HEADING ANGLE IN DEGREES HERE
TestCaseCOOR = [8,8] # <---------------TESTER PUT INITIAL X,Y COORDINATE PT HERE
XCOOR1 = TestCaseCOOR[0]
XCOOR1 = XCOOR1 * scaling
YCOOR1 = TestCaseCOOR[1]
YCOOR1 = YCOOR1 * scaling
TestCaseXY = [XCOOR1,YCOOR1, Theta_Start]  
RPM1 = 5  # <------------TESTER PUT RPM 1 VALUE HERE
RPM2 = 10 # <------------TESTER PUT RPM 2 VALUE HERE
FinalStateCOOR = [9,9] # <---------------TESTER PUT GOAL X,Y COORDINATE PT HERE
XCOOR2 = FinalStateCOOR[0]
XCOOR2 = XCOOR2 * scaling
YCOOR2 = FinalStateCOOR[1]
YCOOR2 = YCOOR2 * scaling
FinalStateXY = [XCOOR2, YCOOR2]   
total_clearance = 0.125 * scaling # gives a clearance of 125 m in the scaled space

# Test Case Initial State:  [x,y] format
# y = rows.... row 0 = y 300
# v = cols.... col 0 = x 0
y = yscale - TestCaseXY[1]
x = TestCaseXY[0]
initial_state = [x, y, Theta_Start]  # pixel coors
start_node_cpy = [x, y, Theta_Start]  # used for plotting the backtracking later
initial_stateID = tuple(initial_state)

# Test Case Final State:  [x,y] format
y = yscale - FinalStateXY[1]
x = FinalStateXY[0]
FinalState = [x, y]  # x, y pixel coordinates
global FinalStateID
FinalStateID = tuple(FinalState)  # pixel coors
print("Final state node (x,y): ", FinalStateID)


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
    cv2.circle(mask, (int(FinalState[0]), int(FinalState[1])), 1, (255, 0, 0), 5)
    cv2.circle(mask, (int(initial_state[0]), int(initial_state[1])), 1, (255, 0, 0), 5)

    # Draw in Current Node points -- NEEDS PIXEL COORS
    cv2.circle(mask, (int(CurrentNode[0]), int(CurrentNode[1])), 1, color, thickness)

    return mask


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
    x = Node[0]
    y = Node[1]

    if x > (xscale - total_clearance) or y > (
            yscale - total_clearance) or x < 0 + total_clearance or y < 0 + total_clearance:
        return True
    else:
        return False
def in_circle(Node):
    global xscale
    global yscale
    x = Node[0]
    y = Node[1]
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
    x = Node[0]
    y = Node[1]
    y = yscale - y

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

# Initialize the plot in the video frame sequence
global mask
mask = 255 * np.ones((1000 , 1000 , 3), np.uint8)  # mask to plot on
mask = space(initial_state, mask, (0, 0, 0),5)  # initialize the plot
outputVideo.write(mask)  # output frame to video sequence


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
        cv2.imshow("back track", mask)
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

def CheckAction(NewNode, action_cost, CTG, ParentNode, flag_obc, velocity):
    global FinalStateID
    ID = tuple(NewNode)  # get the tuple of child node
    CTC = ctc_map[int((parent_ID[0]) / threshold), int((parent_ID[1]) / threshold)] + action_cost  # total cost for child node
    total_cost = CTC + CTG

    # Obstacle Check
    if flag_obc == False:  # NOT in obstacles

        if ID in VisitedDict and cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] > total_cost:  # if Node has been visited before, ensure it's stored cost is the LOWEST value
            VisitedDict[ID] = parent_ID  # add to visited Dictionary the child/parent
            ctc_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = CTC  # ensure lowest cost is stored on cost map
            cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = total_cost
            VelDict[ID] = (velocity)
        elif ID not in VisitedDict and cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] > total_cost:  # Node hasn't been visited, therefore cost map for this point should be infinity still
            VisitedQ_eachRound.append(NewNode)  # add child to OPEN LIST for future node exploration
            VisitedDict[ID] = parent_ID  # add to visited Dictionary the child/parent
            cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = total_cost  # ensure lowest cost is stored on cost map
            ctc_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = CTC
            VelDict[ID] = (velocity)


def Step1(NewNode, theta_i, UL, UR, ParentNode):
    global radius
    x, y = NewNode[:2]  # unpack the node into pixel x y coords
    t = 0  # start integration timer
    dt = 0.1
    theta_n = 3.14 / 180 * theta_i  #convert from degree to radian 
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

    # get cost info and check action 
    ctc = D  # incremental cost for the action
    ctg = ((NewNode[0] - FinalState[0]) ** 2 + (NewNode[1] - FinalState[1]) ** 2) ** 0.5  # euclidian distance CTG
    CheckAction(NewNode, ctc, ctg, ParentNode, flag_obc, velocity)  # check the action


def ActionSet(CurrentNode):
    global x
    global y
    global mask
    NewNode = CurrentNode.copy()  # this will become the future child node
    NewNode_copy = CurrentNode.copy()  # this will become the future parent node
    Theta_i = CurrentNode[2]
    actions = [(RPM1, RPM1), (RPM2, RPM2), (RPM2, 0), (0, RPM2), (RPM1, 0), (0, RPM1), (RPM1, RPM2), (RPM2, RPM1)]
    for action in actions:
        Step1(NewNode, Theta_i, action[0], action[1], NewNode_copy)


#############################################################################################
# Follow the flow chart
#############################################################################################

print("\nInitial State node  ", initial_stateID)
cost_map = np.inf * np.ones((int((10 * scaling)/ threshold), int((10 * scaling) / threshold)))  # initialize cost map where each node = infinity
cost_map[int((initial_state[0]) / threshold), int((initial_state[1]) / threshold)] = 0  # cost at initial state = 0
ctc_map = np.inf * np.ones((int((10 * scaling)/ threshold), int((10 * scaling) / threshold)))
ctc_map[int((initial_state[0]) / threshold), int((initial_state[1]) / threshold)] = 0  # cost at initial state = 0
new_cost = 0

VelDict[tuple(initial_state[:3])] = (0,0)
VisitedDict[tuple(initial_state[:3])] = 'Initial State'  # add initial state into Visited Data Structure
VisitedQ_eachRound.append(initial_state)  # add initial state [x,y] pix coors to OPEN LIST
Checkit = 0
ID = tuple(initial_state[:2])  # tuple pixel coors

min = 0  # initialize finding the lowest cost node
i = 0  # initialize finding the lowest cost node

while ID != FinalStateID:

    if len(VisitedQ_eachRound) > 1:

        for i in range(0, len(VisitedQ_eachRound) - 1):  # for each node in OPEN LIST

            if cost_map[int((VisitedQ_eachRound[min][0]) / threshold), int((VisitedQ_eachRound[min][1]) / threshold)] > \
                    cost_map[int((VisitedQ_eachRound[i][0]) / threshold), int(
                        (VisitedQ_eachRound[i][1]) / threshold)]:  # choose lowest cost
                min = i  # choose node to be explored = lowest costing one

    Checkit = VisitedQ_eachRound.pop(min)  # remove lowest costing node from OPEN LIST and explore it
    VisitedQ.enqueue(Checkit)  # add this node to the CLOSED Q
    closed_list.append(Checkit)  # add node to closed list
    ID = tuple(Checkit[:3])  # tuple pixel coors
    angle = tuple(Checkit[2:3])
    angle_temp = Checkit[2]
    parent_ID = ID  # Make node a parent


    #Distance to Goal Check
    a = ID[0] - FinalStateID[0]
    b = ID[1] - FinalStateID[1]
    DistToGoal = (a ** 2 + b ** 2) ** 0.5

    if DistToGoal <= 15:
        print("\nEuclidean Distance to Goal= ", DistToGoal)
        print("Goal reached at ID (x,y,theta) ", ID)
        roadmap(ID)  # compute the roadmap from initial to final state

 
    ActionSet(Checkit)  # Get the children
