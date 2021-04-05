import numpy as np
import cv2
import sys
import datetime
import math

starttime = datetime.datetime.now()
print("start time: ", starttime)

#############################################################################################
# Initial / Final State Info  ********TESTER INPUT NEEDED HERE FOR START/GOAL POINTS**********
#############################################################################################

# Initializing some values
global ctr
ctr = 0
global closed_list
closed_list = []
global start_node_cpy
global yscale
global xscale
yscale = 300
xscale = 400
global theta_adaptive_n
global theta_adaptive



# Test Case Initial State:  [x,y] format
# Get pixel x,y coordinates of initial state
TestCaseXY = [370, 50]  # <--------------------------TESTER PUT INITIAL X,Y COORDINATE PT HERE
# y = rows.... row 0 = y 300
# v = cols.... col 0 = x 0
y = yscale - TestCaseXY[1]
x = TestCaseXY[0]
initial_state = [x, y]  # pixel coors
start_node_cpy = [x, y] #used for plotting the backtracking later
initial_stateID = tuple(initial_state)
print("Test Case (x,y) starting point is: \n", TestCaseXY, file=open("NodePath.txt", "w"))



# Test Case Final State:  [x,y] format
FinalStateXY = [30, 50]  # <--------------------------TESTER PUT GOAL X,Y COORDINATE PT HERE
y = yscale - FinalStateXY[1]
x = FinalStateXY[0]
FinalState = [x, y]  # x, y pixel coordinates
global FinalStateID
FinalStateID = tuple(FinalState)  # pixel coors
print("Final state node (x,y): ", FinalStateID)



#Find which way the graph will search (left <-- or right -->)
theta_adaptive = 0
temp_x = FinalState[0] - initial_state[0]
temp_y = FinalState[1] - initial_state[1]

if temp_x >= 0 and temp_y >= 0:
    theta_adaptive_n = 0

elif temp_x < 0 and temp_y >= 0:
    theta_adaptive_n = 180

elif temp_x < 0 and temp_y < 0:
    theta_adaptive_n = 180

elif temp_x >= 0 and temp_y < 0:
    theta_adaptive_n = 0


print("\nTest Case (x,y) goal point is: \n", FinalStateXY,"\n\nThe nodes are stored as a tuple of (x,y) PIXEL coordinates, i.e. the Final State node is ", FinalStateID, file=open("NodePath.txt", "a"))



# Robot size and movement details
global radius
global clearance
global theta
global step
global threshold
global total_clearance
radius = 10  # robot radius in pixels
clearance = 5  # robot clearance in pixels
theta = 30  # in degrees, angle between each step size
step = 10  # magnitude of mvmt in units 1<=d<=10
threshold = 0.5  # *** THIS IS NOT CONFIGURABLE. DO NOT CHANGE
total_clearance = radius + clearance




# Initialize Visited Dictionary
VisitedDict = {}

#############################################################################################
# Video writer info
#############################################################################################

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 50
outputVideo = cv2.VideoWriter("Sweeping_Phase_2.mp4", fourcc, fps,(xscale, yscale))  # output resolution must match input frame (60% resized from 1980x1020)
print("\n...Creating Video...\n")


#############################################################################################
# Plot -- READS PIXEL COORS and plots pixel coors
#############################################################################################

def space(CurrentNode, mask, color):
    global FinalState  # PIXEL COORS
    global initial_state  # pixel coors

    # Always draw in Obstacle shapes
    cv2.circle(mask, (90, yscale - 70), 35, (128, 255, 0), 2)
    cv2.ellipse(mask, (246, yscale - 145), (60, 30), 0, 0, 360, (128, 255, 0), 2)
    rect = np.array([[[48, yscale - 108], [36, yscale - 125], [159, yscale - 211], [171, yscale - 194]]], np.int32)
    C_shape = np.array([[[200, yscale - 280], [230, yscale - 280], [230, yscale - 270], [210, yscale - 270],
                         [210, yscale - 240], [230, yscale - 240], [230, yscale - 230], [200, yscale - 230]]], np.int32)
    cv2.polylines(mask, rect, True, (0, 255, 0), 2)
    cv2.polylines(mask, C_shape, True, (0, 255, 0), 2)

    # Always draw in Initial/Final State points
    cv2.circle(mask, (int(FinalState[0]), int(FinalState[1])), 1, (255, 0, 0), 5)
    cv2.circle(mask, (int(initial_state[0]), int(initial_state[1])), 1, (255, 0, 0), 5)

    # Draw in Current Node points -- NEEDS PIXEL COORS
    cv2.circle(mask, (int(CurrentNode[0]), int(CurrentNode[1])), 1, color, 1)

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
    r = 35

    # if inside circle, return true
    if (x - 90) ** 2 + (y - (yscale - 70)) ** 2 <= (r + total_clearance) ** 2:
        return True
    else:
        return False
def in_oval(Node):
    global xscale
    global yscale
    x = Node[0]
    y = Node[1]
    a = 60
    b = 30

    # if inside oval, return true
    if ((x - 246) ** 2) / ((a + total_clearance) ** 2) + ((y - (yscale - 145)) ** 2) / (
            (b + total_clearance) ** 2) <= 1:
        return True
    else:
        return False
def in_rectangle(Node):
    global xscale
    global yscale
    x = Node[0]
    y = Node[1]

    x1 = 48
    y1 = yscale - 108
    x2 = 36
    y2 = yscale - 125
    x3 = 159
    y3 = yscale - 211
    x4 = 171
    y4 = yscale - 194

    # fit 4 lines to make up the rectangle
    # polyfit returns slope and y-int coeff's for linear polynomal
    Mb_1 = np.polyfit([x1, x2], [y1, y2], 1)
    Mb_2 = np.polyfit([x2, x3], [y2, y3], 1)
    Mb_3 = np.polyfit([x3, x4], [y3, y4], 1)
    Mb_4 = np.polyfit([x4, x1], [y4, y1], 1)

    # find where Node is in relation to each side of rectangle
    side1 = y - (Mb_1[0] * (x)) - (Mb_1[1] + total_clearance)
    side2 = y - (Mb_2[0] * (x)) - (Mb_2[1] - total_clearance)
    side3 = y - (Mb_3[0] * (x)) - (Mb_3[1] - total_clearance)
    side4 = y - (Mb_4[0] * (x)) - (Mb_4[1] + total_clearance)

    # if inside rectangle, return trus
    if side1 <= 0 and side2 >= 0 and side3 >= 0 and side4 <= 0:
        return True
    else:
        return False
def in_C_shape1(Node):
    global xscale
    global yscale
    x = Node[0]
    y = Node[1]

    x1 = 200 - total_clearance
    x2 = 230 + total_clearance

    # if inside rectangle, return true
    if (x1 <= x <= x2 and (60 - total_clearance) <= y <= (70 + total_clearance)) or (
            x1 <= x <= x2 and (20 - total_clearance) <= y <= (30 + total_clearance)) or (
            x1 <= x <= (210 + total_clearance) and 30 <= y <= 60):
        return True
    else:
        return False
def in_obstacles(Node):
    in_rect = in_rectangle(Node)
    in_ovl = in_oval(Node)
    in_circ = in_circle(Node)
    in_C1 = in_C_shape1(Node)
    out_of_bound = out_of_bounds(Node)

    if in_rect == True or in_ovl == True or in_circ == True or in_C1 == True or out_of_bound == True:
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
mask = 255 * np.ones((300, 400, 3), np.uint8)  # mask to plot on
mask = space(initial_state, mask, (0, 0, 0))  # initialize the plot
outputVideo.write(mask)  # output frame to video sequence


#############################################################################################
# Plot the Optimal Path -- PLOTTING IN PIXEL COORS
#############################################################################################

def Plot_OurMap(OurMap):
    global mask
    t = 0
    
    for EachID in OurMap:
        x = int(EachID[0])
        y = int(EachID[1])
        CurrentNode = [x, y]

        t = t + 1
        if t % 5 == 0:   #note: start_node_cpy was initialized as a copy of the init state node
            cv2.arrowedLine(mask, (start_node_cpy[0], start_node_cpy[1]), (CurrentNode[0], CurrentNode[1]), (255, 0, 0), 3, tipLength=0.30)
            cv2.imshow("backtrack", mask)
            cv2.waitKey(2000)
            outputVideo.write(mask)
            start_node_cpy[0] = CurrentNode[0]   #rename start node using the child of prev point
            start_node_cpy[1] = CurrentNode[1]

    # Video compelte, calculate code run time
    outputVideo.release()
    cv2.destroyAllWindows()
    endtime = datetime.datetime.now()
    runtime = endtime - starttime
    print("\nRun time: ", runtime)

    print("\nDone. Can see results in NodePath.txt and Sweeping_Phase_2.mp4!")
    sys.exit()


#############################################################################################
# Define parent/child roadmap for initial state to goal state --MAP IS PIXEL COORS
#############################################################################################

def roadmap(ID):
    global parent_ID

    OurMap = []  # initialize the roadmap
    OurMap.append(ID)  # start by adding final state tuple
    parent = VisitedDict[ID]  # get parent tuple
    ID = parent  # set parent -> new child

    while parent != 'Initial State':
        OurMap.append(ID)  # add child tuple to road map
        parent = VisitedDict[ID]  # get parent tuple
        ID = parent  # set parent -> new child

        if parent == 'Initial State':
            OurMap.reverse()  # pitstops were grabbed in reverse order (Goal --> Initial), so need to reverse
            break

    print("\nOur Road Map is: ", OurMap, file=open("NodePath.txt", "a"))
    Plot_OurMap(OurMap)


#############################################################################################
# Define 4 functions to move the blank tile in each direction and store the NewNode in a list
#############################################################################################

def CheckAction(NewNode, action_cost, CTG, ParentNode):
    global FinalStateID
    ChildNode = NewNode.copy() #used for plotting the vector
    ID = tuple(NewNode)  # get the tuple of child node
    CTC = ctc_map[int((parent_ID[0]) / threshold), int((parent_ID[1]) / threshold)] + action_cost  # total cost for child node
    total_cost = CTC + CTG
    
    #Obstacle Check
    flag_obc = in_obstacles(NewNode)
    
    if flag_obc == False:   #NOT in obstacles        
        if ID in VisitedDict and cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] > total_cost:  # if Node has been visited before, ensure it's stored cost is the LOWEST value
            VisitedDict[ID] = parent_ID  # add to visited Dictionary the child/parent
            ctc_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = CTC  # ensure lowest cost is stored on cost map
            cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = total_cost

        elif ID not in VisitedDict and cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] > total_cost:  # Node hasn't been visited, therefore cost map for this point should be infinity still
            VisitedQ_eachRound.append(NewNode)  # add child to OPEN LIST for future node exploration
            VisitedDict[ID] = parent_ID  # add to visited Dictionary the child/parent
            cost_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = total_cost  # ensure lowest cost is stored on cost map
            ctc_map[int((ID[0]) / threshold), int((ID[1]) / threshold)] = CTC
            
            
            if ctr % 1 == 0:  #can set how often a vector is plotted (for more visibility in visualizaiton)
                cv2.arrowedLine(mask, (int(ParentNode[0]), int(ParentNode[1])), (int(ChildNode[0]), int(ChildNode[1])), (0,0,255), 1, tipLength=0.5)
                cv2.imshow("Plotting", mask)
                cv2.waitKey(1)
                outputVideo.write(mask)



def Step1(NewNode, ParentNode):
    x, y = NewNode  # unpack the node into pixel x y coords
    Xtheta = 1 * theta
    x_step = step * math.cos(Xtheta + theta_adaptive)
    y_step = step * math.sin(Xtheta + theta_adaptive)
    x_raw = (2 * int(round(x + x_step)))
    x_new = x_raw / 2  # applies the 0.5 threshold
    y_raw = (2 * int(round(y + y_step)))
    y_new = y_raw / 2  # applies the 0.5 threshold

    swap1 = [x_new, y_new]  # perform the action
    NewNode = swap1
    ctc = step  # incremental cost for the action
    ctg = ((NewNode[0] - FinalState[0]) ** 2 + (NewNode[1] - FinalState[1]) ** 2) ** 0.5  # euclidian distance CTG
    CheckAction(NewNode, ctc, ctg, ParentNode)  # check the action
def Step2(NewNode, ParentNode):
    x, y = NewNode  # unpack the node into pixel x y coords
    Xtheta = 2 * theta
    x_step = step * math.cos(Xtheta + theta_adaptive)
    y_step = step * math.sin(Xtheta + theta_adaptive)
    x_raw = (2 * int(round(x + x_step)))
    x_new = x_raw / 2  # applies the 0.5 threshold
    y_raw = (2 * int(round(y + y_step)))
    y_new = y_raw / 2  # applies the 0.5 threshold

    swap1 = [x_new, y_new]  # perform the action
    NewNode = swap1
    ctc = step  # incremental cost for the action
    ctg = ((NewNode[0] - FinalState[0]) ** 2 + (NewNode[1] - FinalState[1]) ** 2) ** 0.5  # euclidian distance CTG
    CheckAction(NewNode, ctc, ctg, ParentNode)  # check the action
def Step3(NewNode, ParentNode):
    x, y = NewNode  # unpack the node into pixel x y coords
    Xtheta = (-1) * theta
    x_step = step * math.cos(Xtheta +theta_adaptive)
    y_step = step * math.sin(Xtheta + theta_adaptive)
    x_raw = (2 * int(round(x + x_step)))
    x_new = x_raw / 2  # applies the 0.5 threshold
    y_raw = (2 * int(round(y + y_step)))
    y_new = y_raw / 2  # applies the 0.5 threshold

    swap1 = [x_new, y_new]  # perform the action
    NewNode = swap1
    ctc = step  # incremental cost for the action
    ctg = ((NewNode[0] - FinalState[0]) ** 2 + (NewNode[1] - FinalState[1]) ** 2) ** 0.5  # euclidian distance CTG
    CheckAction(NewNode, ctc, ctg, ParentNode)  # check the action
def Step4(NewNode, ParentNode):
    x, y = NewNode  # unpack the node into pixel x y coords
    Xtheta = (-2) * theta
    x_step = step * math.cos(Xtheta + theta_adaptive)
    y_step = step * math.sin(Xtheta + theta_adaptive)
    x_raw = (2 * int(round(x + x_step)))
    x_new = x_raw / 2  # applies the 0.5 threshold
    y_raw = (2 * int(round(y + y_step)))
    y_new = y_raw / 2  # applies the 0.5 threshold

    swap1 = [x_new, y_new]  # perform the action
    NewNode = swap1
    ctc = step  # incremental cost for the action
    ctg = ((NewNode[0] - FinalState[0]) ** 2 + (NewNode[1] - FinalState[1]) ** 2) ** 0.5  # euclidian distance CTG
    CheckAction(NewNode, ctc, ctg, ParentNode)  # check the action
def Step5(NewNode, ParentNode):
    x, y = NewNode  # unpack the node into pixel x y coords
    Xtheta = 0 * theta
    x_step = step * math.cos(Xtheta)
    y_step = step * math.sin(Xtheta)
    x_raw = (2 * int(round(x + x_step)))
    x_new = x_raw / 2  # applies the 0.5 threshold
    y_raw = (2 * int(round(y + y_step)))
    y_new = y_raw / 2  # applies the 0.5 threshold

    swap1 = [x_new, y_new]  # perform the action
    NewNode = swap1
    ctc = step  # incremental cost for the action
    ctg = ((NewNode[0] - FinalState[0]) ** 2 + (NewNode[1] - FinalState[1]) ** 2) ** 0.5  # euclidian distance CTG
    CheckAction(NewNode, ctc, ctg, ParentNode)  # check the action


def ActionSet(CurrentNode):
    global x
    global y
    global mask
    NewNode = CurrentNode.copy()       #this will become the future child node
    NewNode_copy = CurrentNode.copy()  #this will become the future parent node

    Step1(NewNode, NewNode_copy)  # PHASE 2 IMPLEMENTATION
    Step2(NewNode, NewNode_copy)
    Step3(NewNode, NewNode_copy)
    Step4(NewNode, NewNode_copy)
    Step5(NewNode, NewNode_copy)



#############################################################################################
# Follow the flow chart
#############################################################################################

# initial_stateID = tuple(initial_state)  # tuple pixel coors
print("\nInitial State node  ", initial_stateID)

cost_map = np.inf * np.ones((int(400 / threshold), int(300 / threshold)))  # initialize cost map where each node = infinity
cost_map[int((initial_state[0]) / threshold), int((initial_state[1]) / threshold)] = 0  # cost at initial state = 0
ctc_map = np.inf * np.ones((int(400 / threshold), int(300 / threshold)))
ctc_map[int((initial_state[0]) / threshold), int((initial_state[1]) / threshold)] = 0  # cost at initial state = 0
new_cost = 0

VisitedDict[initial_stateID] = 'Initial State'  # add initial state into Visited Data Structure
VisitedQ_eachRound.append(initial_state)  # add initial state [x,y] pix coors to OPEN LIST

Checkit = 0
ID = tuple(initial_state)  # tuple pixel coors

min = 0 # initialize finding the lowest cost node
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
    closed_list.append(Checkit) #add node to closed list
    ID = tuple(Checkit)  # tuple pixel coors
    parent_ID = ID  # Make node a parent

    a = ID[0] - FinalStateID[0]
    b = ID[1] - FinalStateID[1]
    DistToGoal = (a ** 2 + b ** 2) ** 0.5

    if DistToGoal <= 1.5:
        print("\nEuclidean Distance to Goal= ", DistToGoal)
        print("Goal reached at ID ", ID)
        roadmap(ID)  # compute the roadmap from initial to final state
    
    ctr = ctr + 1  #used for plotting frequency later

    ActionSet(Checkit)  # Get the children
