import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import random
from matplotlib.patches import Arc
import sys
import copy

class Node():
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.p=None
        self.tan_pt1_x=None
        self.tan_pt1_y=None
        self.tan_pt2_x=None
        self.tan_pt2_y=None
        self.Ti=None
        self.S=None
        self.coord1=None
        self.coord2=None
        self.coord3=None
        self.coord4=None
        self.theta11=None
        self.theta12=None
        self.theta13=None
        self.theta14=None


plt.axis([-1, 21, -1, 21])
                
obstacle_1_x=15
obstacle_1_y=3

obstacle_2_x=4
obstacle_2_y=5

obstacle_rs=1

obs1 = plt.Rectangle((4,9),1,12, fc='r')
plt.gca().add_patch(obs1)

obs2 = plt.Rectangle((15,9),1,12, fc='r')
plt.gca().add_patch(obs2)

obs3 = plt.Rectangle((9.5,-1),1,12, fc='r')
plt.gca().add_patch(obs3)

obs = plt.Circle((obstacle_1_x, obstacle_1_y), radius=obstacle_rs, fc='r')
plt.gca().add_patch(obs)

obs = plt.Circle((obstacle_2_x, obstacle_2_y), radius=obstacle_rs, fc='r')
plt.gca().add_patch(obs)

current_x=2
current_y=15
current_th=0        #current angle in Degrees

PI=3.14
r_min=0.2
step=1

goal_x=18
goal_y=18
goal_th=90           #goal angle in Degrees

start_circle = plt.Circle((current_x, current_y), radius=0.4, fc='b')
plt.gca().add_patch(start_circle)

goal_circle = plt.Circle((goal_x, goal_y), radius=0.4, fc='y')
plt.gca().add_patch(goal_circle)

plt.gca().set_aspect('equal', adjustable='box')

tree = [Node(current_x,current_y)]

def LSL(current_x,current_y,current_th,goal_x,goal_y,goal_th):
    c1_x=current_x-r_min*math.cos(math.radians(current_th)-(PI/2)) #center of first circle of min_turning radius
    c1_y=current_y-r_min*math.sin(math.radians(current_th)-(PI/2))

    c2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(PI/2))  #center of second circle of min_turning radius
    c2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(PI/2))
    theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

    tan_pt1_x=c1_x+r_min*math.cos(theta)  #tangent point in first circle of min_turning radius
    tan_pt1_y=c1_y-r_min*math.sin(theta)

    tan_pt2_x=c2_x+r_min*math.cos(theta)  #tangent point in second circle of min_turning radius
    tan_pt2_y=c2_y-r_min*math.sin(theta)

    S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

    Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) #Initial Turning angle

    Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y) #Final Turning angle

    if Ti<0:
        Ti+=2*np.pi
    if Tf<0:
        Tf+=2*np.pi

    total_dist = Ti*r_min + Tf*r_min + S  #Total Distance from current to Goal

    ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"LL"]

    return ans

def RSR(current_x,current_y,current_th,goal_x,goal_y,goal_th):
    c1_x=current_x+r_min*math.cos(math.radians(current_th)-(PI/2)) #center of first circle of min_turning radius
    c1_y=current_y+r_min*math.sin(math.radians(current_th)-(PI/2))

    c2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(PI/2)) #center of second circle of min_turning radius
    c2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(PI/2))

    theta = (np.arctan2(c2_x-c1_x, c2_y-c1_y))

    tan_pt1_x=c1_x-r_min*math.cos(theta) #tangent point in first circle of min_turning radius
    tan_pt1_y=c1_y+r_min*math.sin(theta)

    tan_pt2_x=c2_x-r_min*math.cos(theta) #tangent point in second circle of min_turning radius
    tan_pt2_y=c2_y+r_min*math.sin(theta)

    S = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5

    Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y) #Initial Turning angle

    Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) #Final Turning angle
    
    if Ti<0:
        Ti+=2*np.pi
    if Tf<0:
        Tf+=2*np.pi

    total_dist = Ti*r_min + Tf*r_min + S  #Total Distance from current to Goal

    Ti= Ti*180/np.pi

    ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RR"]
    return ans

def LSR(current_x,current_y,current_th,goal_x,goal_y,goal_th):
    c1_x=current_x-r_min*math.cos(math.radians(current_th)-(PI/2)) #center of first circle of min_turning radius
    c1_y=current_y-r_min*math.sin(math.radians(current_th)-(PI/2))

    c2_x=goal_x+r_min*math.cos(math.radians(goal_th)-(PI/2)) #center of second circle of min_turning radius
    c2_y=goal_y+r_min*math.sin(math.radians(goal_th)-(PI/2))

    S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5 #Distance from (c1_x,c1_y) to (c2_x,c2_y)

    if(S1**2 - ((2*r_min)**2))<0:
        return None
    
    S = (S1**2 - ((2*r_min)**2))**0.5

    theta =   (np.arctan2((c2_x-c1_x), c2_y-c1_y)) - np.arctan2(r_min,S/2.0)

    tan_pt1_x=c1_x+r_min*math.cos(theta) #tangent point in first circle of min_turning radius
    tan_pt1_y=c1_y-r_min*math.sin(theta)

    tan_pt2_x=c2_x-r_min*math.cos(theta) #tangent point in second circle of min_turning radius
    tan_pt2_y=c2_y+r_min*math.sin(theta)

    Ti = np.arctan2(current_x-c1_x,current_y-c1_y) - np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) #Initial Turning angle

    Tf = np.arctan2(goal_x - c2_x,goal_y- c2_y) - np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) #Final Turning angle

    if Ti<0:
        Ti+=2*np.pi
    if Tf<0:
        Tf+=2*np.pi

    total_dist = Ti*r_min + Tf*r_min + S  #Total Distance from current to Goal

    Ti= Ti*180/np.pi

    ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"LR"]
    return ans

def RSL(current_x,current_y,current_th,goal_x,goal_y,goal_th):
    c1_x=current_x+r_min*math.cos(math.radians(current_th)-(PI/2)) #center of first circle of min_turning radius
    c1_y=current_y+r_min*math.sin(math.radians(current_th)-(PI/2))

    c2_x=goal_x-r_min*math.cos(math.radians(goal_th)-(PI/2)) #center of second circle of min_turning radius
    c2_y=goal_y-r_min*math.sin(math.radians(goal_th)-(PI/2))

    S1 = ((c2_x-c1_x)**2 + (c2_y-c1_y)**2)**0.5 #Distance from (c1_x,c1_y) to (c2_x,c2_y)

    if(S1**2 - ((2*r_min)**2))<0:
        return None
    
    S = (S1**2 - ((2*r_min)**2))**0.5

    theta =   + (np.arctan2((c2_x-c1_x), c2_y-c1_y)) + np.arctan2(r_min,S/2.0)

    tan_pt1_x=c1_x-r_min*math.cos(theta) #tangent point in first circle of min_turning radius
    tan_pt1_y=c1_y+r_min*math.sin(theta)

    tan_pt2_x=c2_x+r_min*math.cos(theta) #tangent point in second circle of min_turning radius
    tan_pt2_y=c2_y-r_min*math.sin(theta)
    
    Ti = np.arctan2(tan_pt1_x-c1_x,tan_pt1_y-c1_y) - np.arctan2(current_x-c1_x,current_y-c1_y) #Initial Turning angle

    Tf = np.arctan2(tan_pt2_x - c2_x,tan_pt2_y- c2_y) - np.arctan2(goal_x - c2_x,goal_y- c2_y) #Final Turning angle

    if Ti<0:
        Ti+=2*np.pi
    if Tf<0:
        Tf+=2*np.pi

    total_dist = Ti*r_min + Tf*r_min + S 

    Ti= Ti*180/np.pi

    ans=[Ti,Tf, total_dist,S,tan_pt1_x,tan_pt1_y,tan_pt2_x,tan_pt2_y,c1_x,c1_y,c2_x,c2_y,"RL"]
    #    0   1.  2.        3. 4.        5.        6.        7.        8.   9.    10.  11
    return ans


def findNearest(sample):
    min_d = sys.maxsize
    ind=0
    min_node=tree[0]
    for indd,node in enumerate(tree):
        dis = ((node.x-sample.x)**2 + (node.y-sample.y)**2)**0.5
        if min_d>dis:
            min_d=dis
            min_node=node
            ind=indd
    return min_node,ind

def RRT():
    j=0
    initial=[]
    theta_end=None
    while True:
        c=0
        j+=1

        sample = Node(random.uniform(0,20),random.uniform(0,20))
        nearestNode,ind = findNearest(sample)
        newNode = copy.deepcopy(nearestNode)
        
        theta = (np.arctan2(sample.y-nearestNode.y, sample.x-nearestNode.x))
        newNode.x+=step*math.cos(theta)
        newNode.y+=step*math.sin(theta)

        theta_start = theta
        theta_end = (np.arctan2(newNode.y-goal_y, newNode.x-goal_x))

        paths=[LSL,RSR,LSR,RSL] 
        minS=sys.maxsize
        ans=[]
        count=0
        curve=""
        ans_count=1
        for i in paths:
            path_ans=i(nearestNode.x,nearestNode.y,theta_start,newNode.x,newNode.y,theta_end)
            count+=1
            if path_ans:
                if minS>path_ans[3]:
                    ans=path_ans
                    minS=path_ans[3]
                    ans_count=count
                    curve=path_ans[12]
        
        if ((4-0.8) < newNode.x < (4+1+0.8) and (9-0.8) < newNode.y < (9+0.8+11.8)):
            c=1

        if ((15-0.8) < newNode.x < (15+1+0.8) and (9-0.8) < newNode.y < (9+0.8+11.8)):
            c=1
         
        if ((9.5-0.8) < newNode.x < (9.5+1+0.8) and -0.8 < newNode.y < (0.8+11.8)):
            c=1 
            
        if ((newNode.x-obstacle_1_x)**2 + (newNode.y-obstacle_1_y)**2)**0.5 < (obstacle_rs+0.8):
            c=1
            
        if ((newNode.x-obstacle_2_x)**2 + (newNode.y-obstacle_2_y)**2)**0.5 < (obstacle_rs+0.8):
            c=1
        
        if c==0:
            plt.plot([ans[4],ans[6]],[ans[5],ans[7]],'-g')

            start_angle1 = np.arctan2(nearestNode.y - ans[9], nearestNode.x - ans[8]) * 180 / np.pi
            end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi

            if curve[0]=="L":
                arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1, color='g')
                newNode.coord1 = ans[8]
                newNode.coord2 = ans[9]
                newNode.theta11 = start_angle1
                newNode.theta12 = end_angle1 
            else:
                arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1, color='g')
                newNode.coord1 = ans[8]
                newNode.coord2 = ans[9]
                newNode.theta11 = end_angle1
                newNode.theta12 = start_angle1
            plt.gca().add_patch(arc1)

            start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
            end_angle2 = np.arctan2(newNode.y - ans[11], newNode.x - ans[10]) * 180 / np.pi

            if curve[1]=="L":
                arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2, color='g')
                newNode.coord3 = ans[10]
                newNode.coord4 = ans[11]
                newNode.theta13 = start_angle2
                newNode.theta14 = end_angle2
            else:
                arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2, color='g')
                newNode.coord3 = ans[10]
                newNode.coord4 = ans[11]
                newNode.theta13 = end_angle2
                newNode.theta14 = start_angle2

            plt.gca().add_patch(arc2)
            plt.pause(0.01)
            plt.gca().set_aspect('equal')

            newNode.p=ind
            newNode.tan_pt1_x=ans[4]
            newNode.tan_pt1_y=ans[5]
            newNode.tan_pt2_x=ans[6]
            newNode.tan_pt2_y=ans[7]
            newNode.S=ans[3]
            newNode.curve=curve
            newNode.Ti=ans[0]
            if j==1:
                initial.append([ans[4],ans[5],ans[6],ans[7],ans[3],curve,ans[0]])

            tree.append(newNode)
            #plt.plot(newNode.x,newNode.y,'ro')
            if ((newNode.x-goal_x)**2 + (newNode.y-goal_y)**2)**0.5 < step:
                break

    paths=[LSL,RSR,LSR,RSL] 
    minS=sys.maxsize
    ans=[]
    count=0
    ans_count=1
    for i in paths:
        path_ans=i(newNode.x,newNode.y,theta_end,goal_x,goal_y,goal_th)
        count+=1
        if path_ans:
            if minS>path_ans[3]:
                ans=path_ans
                minS=path_ans[3]
                ans_count=count
                curve=path_ans[12]

    goalNode = copy.deepcopy(nearestNode)
    
    goalNode.x=goal_x
    goalNode.y=goal_y
    goalNode.tan_pt1_x=ans[4]
    goalNode.tan_pt1_y=ans[5]
    goalNode.tan_pt2_x=ans[6]
    goalNode.tan_pt2_y=ans[7]
    goalNode.S=ans[3]
    goalNode.curve=curve
    goalNode.Ti=ans[0]

    plt.plot([ans[4],ans[6]],[ans[5],ans[7]])

    start_angle1 = np.arctan2(newNode.y - ans[9], newNode.x - ans[8]) * 180 / np.pi
    end_angle1 = np.arctan2(ans[5] - ans[9], ans[4] - ans[8]) * 180 / np.pi
    
    if curve[0]=="L":
        arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=start_angle1, theta2=end_angle1, color='g')
        goalNode.coord1 = ans[8]
        goalNode.coord2 = ans[9]
        goalNode.theta11 = start_angle1
        goalNode.theta12 = end_angle1 
    else:
        arc1 = patches.Arc((ans[8], ans[9]), 2*r_min,2*r_min,theta1=end_angle1, theta2=start_angle1, color='g')
        goalNode.coord1 = ans[8]
        goalNode.coord2 = ans[9]
        goalNode.theta11 = end_angle1
        goalNode.theta12 = start_angle1
    plt.gca().add_patch(arc1)

    start_angle2 = np.arctan2(ans[7] - ans[11], ans[6] - ans[10]) * 180 / np.pi
    end_angle2 = np.arctan2(goal_y - ans[11], goal_x - ans[10]) * 180 / np.pi
    
    if curve[1]=="L":
        arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=start_angle2, theta2=end_angle2, color='g')
        goalNode.coord3 = ans[10]
        goalNode.coord4 = ans[11]
        goalNode.theta13 = start_angle2
        goalNode.theta14 = end_angle2
    else:
        arc2 = patches.Arc((ans[10], ans[11]), 2*r_min,2*r_min,theta1=end_angle2, theta2=start_angle2, color='g')
        goalNode.coord3 = ans[10]
        goalNode.coord4 = ans[11]
        goalNode.theta13 = end_angle2
        goalNode.theta14 = start_angle2

    plt.gca().add_patch(arc2)

    plt.gca().set_aspect('equal')

    path=[[newNode.tan_pt1_x,newNode.tan_pt1_y,newNode.tan_pt2_x,newNode.tan_pt2_y,newNode.S,newNode.curve,newNode.Ti,newNode.coord1,newNode.coord2,newNode.coord3,newNode.coord4,newNode.theta11,newNode.theta12,newNode.theta13,newNode.theta14]]
    while True:
        if not newNode.p:
            break
        path.append([tree[newNode.p].tan_pt1_x,tree[newNode.p].tan_pt1_y,tree[newNode.p].tan_pt2_x,tree[newNode.p].tan_pt2_y,tree[newNode.p].S,tree[newNode.p].curve,tree[newNode.p].Ti,tree[newNode.p].coord1,tree[newNode.p].coord2,tree[newNode.p].coord3,tree[newNode.p].coord4,tree[newNode.p].theta11,tree[newNode.p].theta12,tree[newNode.p].theta13,tree[newNode.p].theta14])
        newNode=tree[newNode.p]


    plt.plot([goalNode.tan_pt1_x,goalNode.tan_pt2_x],[goalNode.tan_pt1_y,goalNode.tan_pt2_y],'-r', linewidth=4)
    arcf1 = patches.Arc((goalNode.coord1, goalNode.coord2), 2*r_min,2*r_min,theta1=goalNode.theta11, theta2=goalNode.theta12, color='r', linewidth=4)
    plt.gca().add_patch(arcf1)
    arcf2 = patches.Arc((goalNode.coord3, goalNode.coord4), 2*r_min,2*r_min,theta1=goalNode.theta13, theta2=goalNode.theta14, color='r', linewidth=4)
    plt.gca().add_patch(arcf2)
    for i in path:
        plt.plot([i[0],i[2]],[i[1],i[3]],'-r', linewidth=4)
        arcf1 = patches.Arc((i[7], i[8]), 2*r_min,2*r_min,theta1=i[11], theta2=i[12], color='r', linewidth=4)
        plt.gca().add_patch(arcf1)
        arcf2 = patches.Arc((i[9], i[10]), 2*r_min,2*r_min,theta1=i[13], theta2=i[14], color='r', linewidth=4)
        plt.gca().add_patch(arcf2)
    plt.show()

st_x,st_y=input('Enter start coordinates - "x y" (range of coordinates 0-20)').split()
start_x=int(st_x)
start_y=int(st_y)

gl_x,gl_y=input('Enter goal coordinates - "x y" (range of coordinates 0-20)').split()
goal_x=int(gl_x)
goal_y=int(gl_y)

    
RRT()
