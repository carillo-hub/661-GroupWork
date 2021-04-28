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
TestCaseCOOR = [8,1] # <---------------------TESTER PUT INITIAL X,Y COORDINATE PT HERE
RPM1 = 10   
RPM2 = 20  
FinalStateCOOR = [0.5,9.5] # <-------------------TESTER PUT GOAL X,Y COORDINATE PT HERE
 

#apply scaling to coors
initX = int(TestCaseCOOR[0] * scaling)
initY = int(TestCaseCOOR[1] * scaling)
TestCaseXY = [initX,initY, Theta_Start]  
print("\nInitial State node cartesian (x,y) ", TestCaseXY)
finX = int(FinalStateCOOR[0] * scaling) 
finY = int(FinalStateCOOR[1] * scaling)
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
            if ((3.75* scaling) <= x_d <= (6.25 * scaling) and (4.25* scaling) <= y_d <= (5.75* scaling))  or ((7.25* scaling) <= x_d <= (8.75* scaling) and (2* scaling) <= y_d <= (4* scaling)) or  ((0.25* scaling) <= x_d <= (1.75* scaling) and (4.25* scaling) <= y_d <= (5.75* scaling)):
                mask[i : i + 1 , j : j +1] = (128, 255, 0)


    # Always draw in Initial/Final State points
    cv2.circle(mask, (int(FinalState[1]), int(FinalState[0])), 1, (0, 0, 255), 10)
    cv2.circle(mask, (int(initial_state[1]), int(initial_state[0])), 1, (0, 0, 255), 10)

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
    y = yscale - Node[0]

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

        mask = space(CurrentNode, mask, (0, 127, 255), 5)
        outputVideo.write(mask)
        cv2.imshow("Plotting", mask)
        cv2.waitKey(1)
            
    # Video compelte, calculate code run time
    outputVideo.release()
    cv2.destroyAllWindows()
    

    if plotting_Viz_only == False:#<--should be true, chenaged for test
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
    #new_vel = VelDict[ID]
    OurVelMap = []
    #OurVelMap.append(new_vel)
    
    new_ID = (ID[0], ID[1])
    OurMap = []  # initialize the roadmap
    OurMap.append(ID)  # start by adding final state tuple
    parent = VisitedDict[new_ID]  # get parent tuple
    ID = parent  # set parent -> new child

    while parent != 'Initial State':
        OurMap.append(ID)  # add child tuple to road map
        #new_vel = VelDict[ID]
        #OurVelMap.append(new_vel)
        parent = VisitedDict[ID]  # get parent tuple
        ID = parent  # set parent -> new child

        if parent == 'Initial State':
            OurMap.reverse()  # pitstops were grabbed in reverse order (Goal --> Initial), so need to reverse
          #  OurVelMap.reverse()
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
# Some RRT functions 
#############################################################################################

def create_Cspace(q_0, q_f):
    #make Cspace dense sequnce list 
    Cspace = []  #dense seq of 1000x1000 possible nodes to sample 
    for n in range (0,10*scaling):
        for m in range (0,10*scaling):
            alpha = (n,m)
            Cspace.append(tuple(alpha))
    Cspace.remove(q_0)   #dont include q_0 in available sample population 
    Cspace.remove(q_f)   #dont include q_f in available sample population 
    return Cspace


def get_random_sample(Cspace):
    #random sample alpha_i
    sample = random.randint(0,len(Cspace))  #index for random sample
    alpha_i = Cspace.pop(sample)   #pops random coors from Cspace = (x,y)
    
    #obstacle detection for alpha_i 
    while in_obstacles(alpha_i) == True: 
        sample = random.randint(0,len(Cspace))  #index for random sample
        alpha_i = Cspace.pop(sample)   #pops random coors from Cspace = (x,y)
        if in_obstacles(alpha_i) == False:
            break 
    return alpha_i


def check_tree_vertexes(alpha_i):
    #check vertexes of RRT tree for q nearest node
    min_dist = np.inf  
    for point in range(0,len(RRT)):    #for all nodes in RRT tree...
        a = alpha_i[0] - RRT[point][0] #find euclidean dist from each tree node to alpha_i 
        b = alpha_i[1] - RRT[point][1] 
        dist = (a ** 2 + b ** 2) ** 0.5
        if dist < min_dist:  #if this euclid dist < min euclid dist...
            min_dist = dist  #reset the min euclid dist
            q_nearest = RRT[point]  #reset the nearest node 
    return min_dist, q_nearest
        

def check_tree_edges(alpha_i, min_dist, q_nearest):
    
    changed = 0  #use this to see if the q_nearest was changed after subsequent edge check 
    for points in range(0,len(edges)):  #check all edge nodes to see if there is a closer q_nearest
        c = alpha_i[0] - edges[points][0] #calc euclid dist from edge node to alpha_i 
        d = alpha_i[1] - edges[points][1]
        dist = (c ** 2 + d ** 2) ** 0.5
        if dist < min_dist: #if have a new min_dist...
            min_dist = dist   #reset the min euclid dist
            q_new = edges[points]  #reset the nearest node
            changed = 1   #flag to indicate an edge node was selected as q_nearest 
    if changed == 1:             #if q_nearest is an edge --> edge node becomes a vertex
        edges.remove(q_new)  #remove q_nearest edge node from edge list 
        RRT.append(q_new)    #add q_nearest as vertex to vertex list
        return q_new
    else: return q_nearest
   

def get_edge(alpha_i, q_nearest):
    
    edge = np.polyfit([alpha_i[1], q_nearest[1]], [alpha_i[0], q_nearest[0]], 1)  #construct edge from q_nearest to alpha_i 
    if alpha_i[1]+1 < q_nearest[1]+1: #set up x-low/x-high range for scanning the edge nodes
        low = alpha_i[1]+1 
        high = q_nearest[1]
    else: 
        high = alpha_i[1]
        low = q_nearest[1] + 1
    
    edge_open_list = []  #open edge list for scanning 
    for e in range(low,high):  #take all x pts between two nodes 
        f = (edge[0] * (e)) + (edge[1])  #y=mx+b, find y 
        edge_node = (int(f),e)  #get each edge node 
        edge_open_list.append(edge_node)  #add edge node to an open edge node list 
    return edge_open_list
 
   
def order_open_list(edge_open_list, q_nearest):
    
    #ensure that you add new edges from Tree (q_nearest) --> alpha_i  (branching outwards)
    if len(edge_open_list) > 1:
        front = edge_open_list[0] #consider the first edge in the list 
        back = edge_open_list[-1] #consider the last edge in the list 
        a0 = front[0] - q_nearest[0] #calc euclid dist from q_nearest to the first/last edge in list
        b0 = front[1] - q_nearest[1]
        a1 = back[0] - q_nearest[0]
        b1 = back[1] - q_nearest[1]
        dist0 = (a0 ** 2 + b0 ** 2) ** 0.5
        dist1 = (a1 ** 2 + b1 ** 2) ** 0.5
        if dist1 < dist0: edge_open_list = sorted(edge_open_list, reverse=True) #list order should be reversed to branch from q_nearest outwards
   
    return edge_open_list


def collision_free_edges(edge_open_list, alpha_i, q_nearest):
     
     #Collision Detection for the whole edge/branch ...trim branch if in obst
     trim = 0
     for each in range(0,len(edge_open_list)): #for each edge node in open list starting from q_nearest --> branching out
         if in_obstacles(edge_open_list[each]) == True:  #check if the edge node is in obst
             clean_edge_open_list = edge_open_list[:each] #cut off the rest of the edge open list (rest of the branch) after collision det
             trim = 1
             if clean_edge_open_list == []: 
                 print("dead branch")
                 return alpha_i, clean_edge_open_list

             else:
                 alpha_i = clean_edge_open_list[-1] #have to reset alpha_i as the tip of the branch
                 clean_edge_open_list = clean_edge_open_list[:-1]  #remove alpha_i from edge list
                 if alpha_i in Cspace:
                     Cspace.remove(alpha_i) #remove alpha_i from Cspace list 
                 return alpha_i, clean_edge_open_list
     if trim == 0:     
         clean_edge_open_list = edge_open_list
         return alpha_i, clean_edge_open_list


def draw_it(q_nearest, alpha_i, mask):
    space(alpha_i, mask, (127,0,0), 8) #can draw alpha_i as a node on the map now
    space(q_nearest, mask, (127,0,0), 8) #can draw q_nearest as a node on the map now
    pt1 = (q_nearest[1],q_nearest[0])  #for plotting, pt = (col, row) 
    pt2 = (alpha_i[1],alpha_i[0])
    cv2.line(mask, pt1, pt2, color = (127,0,127), thickness=2)
    outputVideo.write(mask)
    cv2.imshow("Plotting", mask)
    cv2.waitKey(1)
    
  
    
    
    
    
    
#############################################################################################
# Check if we have a straight path from start to goal node or from sample node to goal node 
#############################################################################################


def clean_path_check(q_0, q_f): 
    
    edge_open_list = get_edge(q_f, q_0)
 
    #Collision Detection for the edge 
    not_straight = 0
    for each in range(0,len(edge_open_list)): #for each edge node in open list starting from q_nearest --> branching out
        if in_obstacles(edge_open_list[each]) == True:  #check if any edge node is in obst
            not_straight += 1
            
    return not_straight, edge_open_list
    
    
     
    
#############################################################################################
# Setup the initial and final state nodes into RRT 
#############################################################################################

#setup initial and final state nodes into data structures 
q_0 = tuple(initial_state[:2])  # tuple pixel coors (yscaled,x) = (row,col)
VisitedDict[tuple(initial_state[:2])] = 'Initial State'
q_f = FinalStateID # tuple pixel coors (yscaled,x) = (row,col)
global RRT
RRT = []  #this is the tree generated by RRT 
RRT.append(q_0)
theta = Theta_Start
global edges
edges = []  #master list of all nodes in all edges






#first check if straight path exists from start to goal 
not_straight, edge_open_list = clean_path_check(q_0, q_f)
if not_straight == 0:
    VisitedDict[q_f] = q_0  #make start node in tree the parent of goal
    RRT.append(q_f)   
    draw_it(q_0, q_f, mask)
    cv2.waitKey(5)
    print("done")
    sys.exit()
else: print("no straight path from start to goal, starting RRT")





#############################################################################################
# Setup the iterations for RRT 
#############################################################################################
once = 0

#make Cspace dense sequnce list of 1000x1000
Cspace = create_Cspace(q_0, q_f)

# RRT sampling iterations
for i in range (0,200):
    
    #random sample alpha_i
    alpha_i = get_random_sample(Cspace)


    #find q_nearest in the tree
    min_dist, q_nearest = check_tree_vertexes(alpha_i)
    q_nearest = check_tree_edges(alpha_i, min_dist, q_nearest)
    

    #add all nodes in new branch to edges list
    edge_open_list = get_edge(alpha_i, q_nearest) 
    edge_open_list = order_open_list(edge_open_list, q_nearest)
    
    alpha_i, clean_edge_open_list = collision_free_edges(edge_open_list, alpha_i,q_nearest)
    

    #ready to end this iteration 
    if clean_edge_open_list != []: 
        for clean in clean_edge_open_list: edges.append(clean) #can now add the open edge list to the master edge list 
        
        VisitedDict[alpha_i] = q_nearest  #make nearest node in tree the parent of alpha_i
        if alpha_i not in RRT: RRT.append(alpha_i)            #add alpha_i to RRT tree now as a vertex   
        print("vist dict child ",alpha_i," = parent ",q_nearest)
        #draw new edge and vertex
        draw_it(q_nearest, alpha_i, mask)
    
        #if haven't connected to goal node yet, try to do so
        if once == 0:
            not_straight, edge_open_list = clean_path_check(alpha_i, q_f)
            if not_straight == 0:
                for each in range(0,len(edge_open_list)): 
                    edges.append(edge_open_list[each])
                draw_it(alpha_i, q_f, mask)
                VisitedDict[q_f] = alpha_i  #make nearest node in tree the parent of alpha_i
                if alpha_i not in RRT: RRT.append(alpha_i)            #add alpha_i to RRT tree now as a vertex   
                RRT.append(q_f)
                once = 1
                #i = 200
                roadmap(q_f)
                
                      
#perform Astar on RRT vertexes
                   
            
  














