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
d_safe = 0.32 #in meters circle radius plus some extra offset
eps = 0.15        #in meters


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



def H_inv(Controller, yaw_angle):

    theta = yaw_angle  #actual yaw value

    B = np.array([[1, 0],
                  [0, 0],
                  [0, 1]])

    ux_bar = Controller[0]
    uy_bar = Controller[1]

    uxuy = np.array([[ux_bar],
                     [uy_bar]])


    theta_array = np.array([[np.cos(theta), np.sin(theta)],
                            [-np.sin(theta), np.cos(theta)]])

    l_array = np.array([[1,0],
                        [0,1/Robot_radius]])

    u = np.matmul(np.matmul(l_array,theta_array),uxuy)

    new_Controller = np.matmul(B, u)
    
    return new_Controller


def Ugtg_controller(desired_state, robot_state):
    v0 = 0.35  #m/s  max vel is given as (vx**2 + vy**2)**0.5 = 0.5 assuming vx=vy and solving it we get 0.5/2**0.5 = 0.35
    Beta = 0.41

    error_gtg = desired_state-robot_state
    error_gtg = np.mean(error_gtg)
    
    #Kg = v0*(1 - np.exp(-Beta*np.absolute(error_gtg)))/np.absolute(error_gtg)        
    Kg = min(max(v0*(1 - np.exp(-Beta*np.absolute(error_gtg)))/np.absolute(error_gtg), 0), 0.5)
    Ugtg = Kg*(desired_state-robot_state).flatten()
    return Ugtg



def Uavo_controller(filtered_obst_pts, sensors_dist, robot_state): #yaw_angle
    #C = 0.5
    C = 4
    #Ko = 0.5
    Uavo_detected = np.zeros((1,2), dtype=float)

    for i in range(len(filtered_obst_pts)):
    
        error_avo =  robot_state-filtered_obst_pts[i]
        error_avo = np.mean(error_avo)
        
        Ko = min(max((1/np.absolute(error_avo))*(C/((np.absolute(error_avo)**2)+eps)), 0), 0.1)
        
        Uavo = Ko*(robot_state-filtered_obst_pts[i])   #x-xo

        #print("Ko: ",Ko, "state: ",1/np.absolute(error_avo), "err: ",np.absolute(error_avo))

        #Weight = (1/min(max(sensors_dist[i], 0.21), 1))

        Weight = 6 
        Uavo = Weight*Uavo

        #print("Uavo:",Uavo)

        Uavo_detected = np.append(Uavo_detected, [Uavo], axis=0)

    #print()
    Uavo_detected = np.delete(Uavo_detected, 0, axis = 0)  #delete zeros
    
    Uavo_summed = np.mean(Uavo_detected, axis=0).flatten()

    return Uavo_summed


def Uwf_controller(Uavo_summed):

    Kwf = 4

    Uavo = Uavo_summed

    Rot_clock = np.array(( (0,   1),
                           (-1,  0) ))  #2x2

    mul_1 = Rot_clock.dot((Uavo[0],Uavo[1])) #had to do because we need only x and y no theta
    Ucwf = np.dot(Kwf, mul_1)

    Rot_Cclock = np.array(( (0,   -1),
                            (1,    0) ))  #2x2

    mul_2 = Rot_Cclock.dot((Uavo[0],Uavo[1])) #had to do because we need only x and y no theta
    Uccwf = np.dot(Kwf, mul_2)

    return Ucwf, Uccwf


def angle_between_2_vectors(vector_1, vector_2):

    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)

    arc = np.arccos(dot_product)
    return arc





def move2coord(Dbug):

    robot_state = init_state.copy() # numpy array for [px, py, theta]
    
    robot_state[0] = pose_tf.transform.translation.x
    robot_state[1] = pose_tf.transform.translation.y
    yaw_angle = yaw # theta

    desired_state = np.array([0., 0.]) # numpy array for goal / the desired [px, py]
    desired_angle = 0.                 #desired  theta
    current_input = np.array([0., 0., 0.]) # initial numpy array for [vx, vy, omega]
    
    global goal_pose
    goal_pose = TransformStamped()     #"""Moves the turtle to the goal."""


    desired_state[0] = float(input("Set your x goal: ")) # Get the input from the user.
    desired_state[1] = float(input("Set your y goal: "))
    
    goal_pose.transform.translation.x = desired_state[0]   #robot_state X value  
    goal_pose.transform.translation.y = desired_state[1]

    #distance_tolerance = float(input("Set your tolerance: "))     #insert a number slightly greater than 0 (e.g. 0.01).
    distance_tolerance = 0.01
    deg_tolerance=0.1
    vel_msg = Twist()

    Stored_robot_state = robot_state #np.array([0., 0, 0.])#robot_state

    while True:

        sensor_Dist = np.array(ranges)
        sensor_Dist = np.clip(sensor_Dist, 0.0, 1.0)
    	
        obst_points = compute_sensor_endpoint(robot_state, yaw_angle, sensor_Dist) # compute sensor reading endpoint
        filtered_obst_pts = np.zeros((1,2), dtype=float)

        for i in range(len(obst_points[0])):
            filtered_obst_pts=np.append(filtered_obst_pts, [[obst_points[0][i],obst_points[1][i]]], axis=0)

        filtered_obst_pts = np.delete(filtered_obst_pts, 0, axis = 0)

#        print("filtered_obst_pts: ",filtered_obst_pts)
#        print()
#        print()
        
        Ugtg = Ugtg_controller(desired_state, robot_state)
        Uavo = Uavo_controller(filtered_obst_pts, sensor_Dist, robot_state)
        Ucwf, Uccwf = Uwf_controller(Uavo)


        Condition_1 = any((element >= d_safe-eps and element <= d_safe+eps)  for element in sensor_Dist)  #sensor dist assumed as x-xo

        Condition_2 = np.dot([Ugtg[0], Ugtg[1]], Ucwf)  # Should be > 0

        Condition_2_angle_degree = np.rad2deg(angle_between_2_vectors([Ugtg[0], Ugtg[1]], Ucwf))

        if (Condition_2_angle_degree>90):
            Condition_2_angle_degree = 180 - Condition_2_angle_degree

        Condition_2 = Condition_2_angle_degree

        Condition_3_angle_degree = np.rad2deg(angle_between_2_vectors([Ugtg[0], Ugtg[1]], Uccwf)) #angle should be < 90
        Condition_3 = Condition_3_angle_degree
 
        direction_angle_degree = np.rad2deg(angle_between_2_vectors([Uavo[0], Uavo[1]], [Ugtg[0], Ugtg[1]]))
        if (direction_angle_degree > 90):
            direction_angle_degree = 180 - direction_angle_degree

        Condition_4 = direction_angle_degree #angle should be < 90
 
        Condition_5 = np.linalg.norm(robot_state-desired_state) <= np.linalg.norm(Stored_robot_state - desired_state) 
        Condition_6 = any((element < d_safe-eps)  for element in sensor_Dist)  #sensor dist assumed as x-xo
        Condition_7 = np.linalg.norm(robot_state-desired_state) < 0.01

        #print("C1_safe:",Condition_1,"C2_clock:",Condition_2 < 90,"C3_CClock:",Condition_3 > 90,"C4_gtg:",Condition_4<90,"C5_gtg2:",Condition_5,"C6:",Condition_6)

        if (Condition_7 == True):    #Reached
                print("DONE:")
                
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.angular.z = 0

        elif (Condition_1 == True and (Condition_2 < 90) == True):  #switch to Ucwf

            #print("UWF_Clk")
            if (Condition_6 == True):
                print("UWF_Clk-AVO")
                Stored_robot_state = robot_state
                Uavo_apply = H_inv(Uavo, yaw_angle)
                vel_msg.linear.x = -Uavo_apply[0]
                vel_msg.linear.y = -Uavo_apply[1]
                vel_msg.angular.z = -Uavo_apply[2]
                
                
            else:
                print("UWF_Clk")
                
                Ucwf_apply = H_inv(Ucwf, yaw_angle)
                vel_msg.linear.x = Ucwf_apply[0]
                vel_msg.linear.y = Ucwf_apply[1]
                vel_msg.angular.z = Ucwf_apply[2]

        elif (Condition_1 == True and (Condition_3 > 90) == True):  #switch to Uccwf
            #print("UWF_CC")
            if (Condition_6 == True):
                print("UWF_CC-AVO")
                Stored_robot_state = robot_state
                Uavo_apply = H_inv(Uavo, yaw_angle)
                vel_msg.linear.x = -Uavo_apply[0]
                vel_msg.linear.y = -Uavo_apply[1]
                vel_msg.angular.z = -Uavo_apply[2]
                
            else:
                print("UWF_CC")
                
                Uccwf_apply = H_inv(Uccwf, yaw_angle)
                vel_msg.linear.x = Uccwf_apply[0]
                vel_msg.linear.y = Uccwf_apply[1]
                vel_msg.angular.z = Uccwf_apply[2]
                

        elif ((Condition_4 < 90) and Condition_5 == True): #GTG
            #print("GTG")
            if (Condition_6 == True):
                print("GTG-AVO")
                Stored_robot_state = robot_state
                Uavo_apply = H_inv(Uavo, yaw_angle)
                
                vel_msg.linear.x = -Uavo_apply[0]
                vel_msg.linear.y = -Uavo_apply[1]
                vel_msg.angular.z = -Uavo_apply[2]
                
            else:
                print("GTG")
                Stored_robot_state = robot_state

                Ugtg_apply = H_inv(Ugtg, yaw_angle)
                vel_msg.linear.x = Ugtg_apply[0]
                vel_msg.linear.y = Ugtg_apply[1]
                vel_msg.angular.z = Ugtg_apply[2]
                
                


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

    


rospy.init_node('turtlebot3_controller', anonymous=True) # Creates a node with name 'turtlebot_controller' and make sure it is a unique node name (using anonymous=True).
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publisher which will publish to the topic '/turtle1/cmd_vel'.
tf_subscriber=rospy.Subscriber('/tf', TFMessage, update_tf)


scan_subscriber = rospy.Subscriber('/scan', LaserScan, update_scan)

pose_tf=TransformStamped()

rate = rospy.Rate(200)
rospy.sleep(1)
#tst()
move2coord(0)

