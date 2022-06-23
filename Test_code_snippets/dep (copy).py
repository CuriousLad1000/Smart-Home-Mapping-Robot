#!/usr/bin/env python
import rospy
#from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist, Point, TransformStamped
#from nav_msgs.msg import Path
from math import pow, atan2, sqrt
from tf2_msgs.msg import TFMessage
import numpy as np

from sensor_msgs.msg import LaserScan



roll = pitch = yaw = 0.0

v0 = 0.5
Beta = 1
Robot_radius = 0.21 #m
d_safe = 0.42 #in meters circle radius plus some extra offset
eps = 0.1        #in meters


ranges = []
dis_foward = 0.0


#init_state = np.array([0., 0., np.pi/2]) # px, py, theta
init_state = np.array([0., 0.]) # px, py
init_ange = 0. #theta

def update_tf(data): #Callback function which is called when a new message of type Pose is received by the subscriber.
    global roll, pitch, yaw
    global pose_tf
    pose_tf = data.transforms[0]

    pose_tf.transform.translation.x = round(pose_tf.transform.translation.x, 4)
    pose_tf.transform.translation.y = round(pose_tf.transform.translation.y, 4)
#    print('pose_tf: ',pose_tf)
    orientation_list = [pose_tf.transform.rotation.x, pose_tf.transform.rotation.y, pose_tf.transform.rotation.z, pose_tf.transform.rotation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
#    print (yaw)
    

def update_scan(readings):
    global ranges,dis_foward
    ranges = readings.ranges
    dis_foward = ranges[359]
    #print(dis_foward)
    #print(len(ranges))




def euclidean_distance(goal):               #Euclidean distance between current pose and the goal.
    return sqrt(pow((goal.transform.translation.x - pose_tf.transform.translation.x), 2) + pow((goal.transform.translation.y - pose_tf.transform.translation.y), 2))


def rad_to_deg(rad):
    deg=(180/np.pi)*rad
#    if rad<0:
        #deg = -deg #if you want to invert the values (eg: -45 becomes +45)
        #deg = 360+deg #only if you want angle in terms of full 360 cycle (eg: -45 turns to 360-45=315)
    return deg

def steering_angle(goal):

    return atan2(goal[1] - pose_tf.transform.translation.y, goal[0] - pose_tf.transform.translation.x)  #<<<<< Mod this 


def angular_vel(goal, constant=4):
    a=steering_angle(goal)
    
    if a<0: #clockwise
       a = np.pi-a

    a = ( (a + np.pi) % (2*np.pi) ) - np.pi # ensure theta within [-pi pi]
    #print('Arc angle:', a)
    #print('Arc angle Degree: ',rad_to_deg(a))
    return constant * (a - yaw)       #<<< Mod This


def HT_matrix(robot_state, yaw_angle, sensors_dist_value, angle):

    rot_sensor_to_body = np.array(( (np.cos(angle), -np.sin(angle), 0),
                                    (np.sin(angle),  np.cos(angle), 0),
                                    (0,              0,             1)))  #3x3
                                    
    Trans_obst_to_sensor = np.array((sensors_dist_value,  0,           1))   #3x1

    #Trans_Obst_to_body = rot_sensor_to_body.dot(Trans_obst_to_sensor)  #dot 
    Trans_Obst_to_body = np.matmul(rot_sensor_to_body, Trans_obst_to_sensor)   #3x1

    rot_robot_to_world = np.array(( (np.cos(yaw_angle), -np.sin(yaw_angle), robot_state[0]),
                                    (np.sin(yaw_angle),  np.cos(yaw_angle), robot_state[1]),
                                    (0,                       0,                                   1) ))  #3x3

    Trans_Robot_to_World = np.matmul(rot_robot_to_world, Trans_Obst_to_body)
    return Trans_Robot_to_World


def compute_sensor_endpoint(robot_state, yaw_angle, sensors_dist):

    # NOTE: we assume sensor position is in the robot's center
    sens_N = len(sensors_dist)
    obst_points = np.zeros((3,sens_N))
    sensor_spacing = 360/sens_N

    for i in range(sens_N):
        #angle = np.pi * 2 * (i*sensor_spacing/360)+yaw_angle  #points position wrt robot's orientation.
        angle = np.pi * 2 * (i*sensor_spacing/360)  #points positions wrt robot's orientation.
        
        Coordinates_XY = HT_matrix(robot_state, yaw_angle, sensors_dist[i], angle)
        
        obst_points[:,i][0] = Coordinates_XY[0]
        obst_points[:,i][1] = Coordinates_XY[1]

#    print("END----------------------------------------------")
    return obst_points[:2,:] # only return x and y values theta neglected






def Ugtg_controller(desired_state, robot_state, yaw_angle):
    v0 = 0.35  #m/s  max vel is given as (vx**2 + vy**2)**0.5 = 0.5 assuming vx=vy and solving it we get 0.5/2**0.5 = 0.35
    Beta = 0.41

    error_gtg = desired_state-robot_state

    error_gtg = np.mean(error_gtg)
    
    #Kg = v0*(1 - np.exp(-Beta*np.absolute(error_gtg)))/np.absolute(error_gtg)        
    Kg = min(max(v0*(1 - np.exp(-Beta*np.absolute(error_gtg)))/np.absolute(error_gtg), 0), 0.05)
      

    theta = yaw_angle  #actual yaw value

    B = np.array([[1, 0],
                  [0, 0],
                  [0, 1]])
    ux_bar = Kg * (desired_state[0] - robot_state[0])
    uy_bar = Kg * (desired_state[1] - robot_state[1])

    uxuy = np.array([[ux_bar],
                     [uy_bar]])

    theta_array = np.array([[np.cos(theta), np.sin(theta)],
                            [-np.sin(theta), np.cos(theta)]])

    l_array = np.array([[1,0],
                        [0,1/Robot_radius]])

    u =np.matmul(np.matmul(l_array,theta_array),uxuy)

    Ugtg = np.matmul(B, u).flatten()
    #print("Ugtg: ",Ugtg)

    return Ugtg



def Uavo_controller(filtered_obst_pts, sensors_dist, robot_state,yaw_angle):
    C = 0.5

    Uavo_detected = np.zeros((1,3), dtype=float)

    for i in range(len(filtered_obst_pts)):
    
        error_avo =  robot_state-filtered_obst_pts[i]       
        error_avo = np.mean(error_avo)
        
        Ko = min(max((1/np.absolute(error_avo))*(C/((np.absolute(error_avo)**2)+eps)), 0), 0.3)
        theta = yaw_angle  #actual yaw value
        B = np.array([[1, 0],
                      [0, 0],
                      [0, 1]])
        ux_bar = Ko * (robot_state[0] - filtered_obst_pts[i][0])
        uy_bar = Ko * (robot_state[1] - filtered_obst_pts[i][1])

        uxuy = np.array([[ux_bar],
                         [uy_bar]])

        theta_array = np.array([[np.cos(theta), np.sin(theta)],
                                [-np.sin(theta), np.cos(theta)]])

        l_array = np.array([[1,0],
                            [0,1/Robot_radius]])

        u =np.matmul(np.matmul(l_array,theta_array),uxuy)

        Uavo = np.matmul(B, u).flatten()

        Weight = 2
        Uavo = Weight*Uavo
  
        #print("Uavo:",Uavo)

        Uavo_detected=np.append(Uavo_detected, [Uavo], axis=0)

    #print()
    Uavo_detected = np.delete(Uavo_detected, 0, axis = 0)  #delete zeros

    Uavo_summed = np.mean(Uavo_detected, axis=0)

    #print("Uavo_summed:",Uavo_summed)
    #print()

    return Uavo_summed


def Uwf_controller(Uavo_summed):

    Kwf = 20

    Uavo = Uavo_summed

    Rot_clock = np.array(( (0,   1),
                           (-1,  0) ))  #2x2

    mul_1 = Rot_clock.dot((Uavo[0],Uavo[1]))
    Ucwf = np.dot(Kwf, mul_1)

    Rot_Cclock = np.array(( (0,   -1),
                            (1,    0) ))  #2x2

    mul_2 = Rot_Cclock.dot((Uavo[0],Uavo[1]))
    Uccwf = np.dot(Kwf, mul_2)

    return Ucwf, Uccwf


def angle_between_2_vectors(vector_1, vector_2):

    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)

    arc = np.arccos(dot_product)
    return arc



def move2coord(Dbug):

    robot_state = init_state.copy() # numpy array for [px, py]
    
    robot_state[0] = pose_tf.transform.translation.x
    robot_state[1] = pose_tf.transform.translation.y
    yaw_angle = yaw # theta

    desired_state = np.array([0., 0.]) # numpy array for goal / the desired [px, py]
    desired_angle = 0.                 #desired  theta

    current_input = np.array([0., 0., 0.]) # initial numpy array for [vx, vy, omega]
    
    global goal_pose
    goal_pose = TransformStamped()     #"""Moves the turtle to the goal."""


    desired_state[0] = 5#float(input("Set your x goal: ")) # Get the input from the user.
    desired_state[1] = 0#float(input("Set your y goal: "))
    
    goal_pose.transform.translation.x = desired_state[0]   #robot_state X value  
    goal_pose.transform.translation.y = desired_state[1]

    #distance_tolerance = float(input("Set your tolerance: "))     #insert a number slightly greater than 0 (e.g. 0.01).
    distance_tolerance = 0.1
    deg_tolerance=0.1
    vel_msg = Twist()

    Stored_robot_state = robot_state  #stores current x,y of bot
    rotate_flag = 1
    Condition_Left = False
    Condition_Right = False
    Condition_Forward = False
    Condition_Back = False
    
    while True:

        sensor_Dist = np.array(ranges)
        sensor_Dist = np.clip(sensor_Dist, 0.0, 1.0)
    	
        obst_points = compute_sensor_endpoint(robot_state, yaw_angle, sensor_Dist) # compute sensor reading endpoint
        filtered_obst_pts = np.zeros((1,2), dtype=float) #This will hold  all reformatted obstacle points in [[X0,Y0],[X1,Y1]...] format

        for i in range(len(obst_points[0])):
            filtered_obst_pts=np.append(filtered_obst_pts, [[obst_points[0][i],obst_points[1][i]]], axis=0)

        filtered_obst_pts = np.delete(filtered_obst_pts, 0, axis = 0)

        Left = filtered_obst_pts[89]
        Right = filtered_obst_pts[269]
        Forward = filtered_obst_pts[0]
        Back = filtered_obst_pts[179]
        
        Ugtg = Ugtg_controller(desired_state, robot_state, yaw_angle)
        Uavo = Uavo_controller(filtered_obst_pts, sensor_Dist, robot_state, yaw_angle)
        Ucwf, Uccwf = Uwf_controller(Uavo)

        if (sensor_Dist[89] >= d_safe-eps and sensor_Dist[89] <= d_safe+eps) : 
            Condition_Left = True 
        else: Condition_Left = False
        
        if (sensor_Dist[269] >= d_safe-eps and sensor_Dist[269] <= d_safe+eps) : 
            Condition_Right = True 
        else: Condition_Right = False
        
        if (sensor_Dist[0] >= d_safe-eps and sensor_Dist[0] <= d_safe+eps) : 
            Condition_Forward = True 
        else: Condition_Forward = False
        
        if (sensor_Dist[179] >= d_safe-eps and sensor_Dist[179] <= d_safe+eps) : 
            Condition_Back = True 
        else: Condition_Back = False
        
        #print("F:", Condition_Forward, "B:", Condition_Back, "L:", Condition_Left, "R:", Condition_Right)
        
        Condition_1 = any((element >= d_safe-eps and element <= d_safe+eps)  for element in sensor_Dist)  #sensor dist assumed as x-xo

        Condition_2 = np.dot([Ugtg[0], Ugtg[1]], Ucwf)  # Should be > 0

        Condition_2_angle_degree = np.rad2deg(angle_between_2_vectors([Ugtg[0], Ugtg[1]], Ucwf))

        #if (Condition_2_angle_degree>90):
        #    Condition_2_angle_degree = 180 - Condition_2_angle_degree

        Condition_2 = Condition_2_angle_degree

        Condition_3_angle_degree = np.rad2deg(angle_between_2_vectors([Ugtg[0], Ugtg[1]], Uccwf)) #angle should be < 90
        Condition_3 = Condition_3_angle_degree
 
        direction_angle_degree = np.rad2deg(angle_between_2_vectors([Uavo[0], Uavo[1]], [Ugtg[0], Ugtg[1]]))
        #if (direction_angle_degree > 90):
        #    direction_angle_degree = 180 - direction_angle_degree

        Condition_4 = direction_angle_degree #angle should be < 90
 
        Condition_5 = np.linalg.norm(robot_state-desired_state) <= np.linalg.norm(Stored_robot_state - desired_state) 
        Condition_6 = any((element < d_safe-eps)  for element in sensor_Dist)  #sensor dist assumed as x-xo
        Condition_7 = np.linalg.norm(robot_state-desired_state) < 0.01
        dis = np.where(sensor_Dist <= d_safe+eps)
        print(dis)
        print()
        #print("C1_safe:",Condition_1,"C2_clock:",Condition_2 < 90,"C3_CClock:",Condition_3 > 90,"C4_gtg:",Condition_4<90,"C5_gtg2:",Condition_5,"C6:",Condition_6)

        if (Condition_7 == True):    #Reached
                print("DONE:")

                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.angular.z = 0

        #elif (Condition_1 == True and (Condition_2 < 90) == True):  #switch to Ucwf
        elif (Condition_1 == True):  #switch to Ucwf

            #print("UWF_Clk")
            if (Condition_6 == True):
                print("UWF_Clk-AVO")
                Stored_robot_state = robot_state #only x and y
                rotate_flag = 1
                
                vel_msg.linear.x = -Uavo[0]
                vel_msg.linear.y = -Uavo[1]
                vel_msg.angular.z = 0
                
            else:
                print("UWF_Clk")
                
                #roate and continue....
                
                if rotate_flag == 1:
                    rotate_flag = 0
                    if (Condition_Forward == False):
                        rotation_Controller(Forward)
                    
                    elif (Condition_Left == False):
                        rotation_Controller(Left)
                    
                    elif (Condition_Right == False):
                        rotation_Controller(Right)
                        
                    elif (Condition_Back == False):
                        rotation_Controller(Back)
                    print("YAW: ", np.rad2deg(yaw_angle))      

                else:
                # move towards, forward sensor
                #calculate new f/w pos. 
                    Ugt_sen = Ugtg_controller(Forward, robot_state, yaw_angle)
                    
                    vel_msg.linear.x = Ugt_sen[0]
                    vel_msg.linear.y = Ugt_sen[1]
                    vel_msg.angular.z = Ugt_sen[2]


        elif ((Condition_4 < 90) and Condition_5 == True): #GTG angle b/w goal and bot less than 90 and bot is closer than before
            #print("GTG")
            if (Condition_6 == True):
                print("GTG-AVO")
                Stored_robot_state = robot_state #only x and y
                rotate_flag = 1

                vel_msg.linear.x = -Uavo[0]
                vel_msg.linear.y = -Uavo[1]
                vel_msg.angular.z = 0#Uavo[2]
                
            else:
                print("GTG")
                Stored_robot_state = robot_state #only x and y
                rotate_flag = 1
                
                vel_msg.linear.x = Ugtg[0]
                vel_msg.linear.y = Ugtg[1]
                vel_msg.angular.z = Ugtg[2]
                
                


        velocity_publisher.publish(vel_msg)         # Publishing our vel_msg
        
        Dist_Err = euclidean_distance(goal_pose)    #>= distance_tolerance
        
        rate.sleep()                                # Publish at the desired rate.

        current_degree = np.rad2deg(yaw)

        if Dbug==1:
            print('Target X: ', goal_pose.transform.translation.x, 'Target Y: ', goal_pose.transform.translation.y)
            print('Current X: ', pose_tf.transform.translation.x, 'Current Y: ', pose_tf.transform.translation.y)
            print("Current Angle (Deg): ",current_degree)            
            print('Distance error: ',Dist_Err)

            print()
        
        if (Dist_Err <= distance_tolerance): #and (average <= deg_tolerance):
            break

        robot_state[0] = pose_tf.transform.translation.x
        robot_state[1] = pose_tf.transform.translation.y
        yaw_angle = yaw

    vel_msg.linear.x = 0      # Stopping our robot after the movement is over.
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.sleep(3)

    #rospy.spin()     # If we press control + C, the node will stop.

    



def rotation_Controller(X_Y_coordinates):
    Av_store = []
    deg_tolerance=0.15
    #global goal_pose
    vel_msg = Twist()
    Av_store.append(1)
    while True:
        # Linear velocity in the x-axis.
#        vel_msg.linear.x = 0
#        vel_msg.linear.y = 0
#        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        #vel_msg.angular.x = 0
        #vel_msg.angular.y = 0
        zz=angular_vel(X_Y_coordinates)    #goal_pose
        average = sum(Av_store) / float(len(Av_store))

        if average > 2:
            zz=zz/average

            vel_msg.angular.z = zz
            velocity_publisher.publish(vel_msg)
 #           rate.sleep()


        vel_msg.angular.z = zz
        
        velocity_publisher.publish(vel_msg)         # Publishing our vel_msg
        Av_store.append(abs(rad_to_deg(zz)))
        rate.sleep()         # Publish at the desired rate.

#        current_degree=(180/np.pi)*yaw
        current_degree = rad_to_deg(yaw)
        #if yaw<0: #clockwise
#            current_degree = 180-current_degree
#            print("minus: ",current_degree)

        if len(Av_store) > 60:           #moving avg.
            Av_store = Av_store[-60:]
        average = sum(Av_store) / float(len(Av_store))
        
        #print()
#        print("Target Angle: ",angl)
#        print("Radian angle:",yaw)
#        print()
#        print('Arc minus yaw Degree',rad_to_deg(zz))
        #print('Current Angle Error (Deg): ',average)
        #print("Current Angle (Deg): ",current_degree)

        if average <= deg_tolerance:
            break

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)




rospy.init_node('turtlebot3_controller', anonymous=True) # Creates a node with name 'turtlebot_controller' and make sure it is a unique node name (using anonymous=True).
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publisher which will publish to the topic '/turtle1/cmd_vel'.
tf_subscriber=rospy.Subscriber('/tf', TFMessage, update_tf)


scan_subscriber = rospy.Subscriber('/scan', LaserScan, update_scan)

pose_tf=TransformStamped()

rate = rospy.Rate(200)
rospy.sleep(1)

move2coord(0)
#tsta()