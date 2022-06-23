#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, TransformStamped
#from nav_msgs.msg import Path
from math import pow, atan2, sqrt
from tf2_msgs.msg import TFMessage
import time
Limit=30
Elaps=0
roll = pitch = yaw = 0.0
PI = 3.1415926535897

def update_tf(data): #Callback function which is called when a new message of type Pose is received by the subscriber.
    global roll, pitch, yaw
    global pose_tf
    pose_tf = data.transforms[0]
    if pose_tf.header.frame_id == "odom":
#       print('pose_tf: ',pose_tf.header.frame_id)
        pose_tf.transform.translation.x = round(pose_tf.transform.translation.x, 4)
        pose_tf.transform.translation.y = round(pose_tf.transform.translation.y, 4)
    #    print('pose_tf: ',pose_tf)
        orientation_list = [pose_tf.transform.rotation.x, pose_tf.transform.rotation.y, pose_tf.transform.rotation.z, pose_tf.transform.rotation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #    print (yaw)

        

def euclidean_distance(goal):               #Euclidean distance between current pose and the goal.
    while True:
        if pose_tf.header.frame_id == "odom":
            return sqrt(pow((goal.transform.translation.x - pose_tf.transform.translation.x), 2) + pow((goal.transform.translation.y - pose_tf.transform.translation.y), 2))


def linear_vel(goal, constant=0.1):
    dist = euclidean_distance(goal)
    dist2=(goal.transform.translation.x)-dist
    #if dist >= (dist2/3) and dist > 0.2:
    if dist2 >= 0.7 and dist > 0.2:
        constant=constant*2
        #print('dist:',dist)
        #print('dist2:',dist2)
    return constant * dist   #constant * euclidean_distance(goal)


def steering_angle(goal):
       while True:
        if pose_tf.header.frame_id == "odom":
            return atan2(goal.transform.translation.y - pose_tf.transform.translation.y, goal.transform.translation.x - pose_tf.transform.translation.x)  #<<<<< Mod this 


def angular_vel(goal, constant=4):
    a=steering_angle(goal)
    return constant * (a - yaw)       #<<< Mod This


def rad_to_deg(rad,cont=0):
    deg=(180/PI)*rad
    if cont==1:
        if rad<0:
            deg=360+deg
            return deg
#    if rad<0:
        #deg = -deg #if you want to invert the values (eg: -45 becomes +45)
        #deg = 360+deg #only if you want angle in terms of full 360 cycle (eg: -45 turns to 360-45=315)
    return deg


def move2coord(x,y,Dbug=0,step=0,TIM=0):
#def move2coord(Dbug):
    flg = False
    global goal_pose
    goal_pose = TransformStamped()     #"""Moves the turtle to the goal."""
#    goal_pose.transform.translation.x = float(input("Set your x goal: "))     # Get the input from the user.
#    goal_pose.transform.translation.y = float(input("Set your y goal: "))
    goal_pose.transform.translation.x = x     # Get the input from the user.
    goal_pose.transform.translation.y = y

    while flg == False:
        Av_store = []
        Av_store.append(1)


        distance_tolerance = 0.1
        deg_tolerance=0.1


        vel_msg = Twist()
        rotation_controller()
        rospy.sleep(1)
        prev = time.time()

        while True:
            time_elapsed = time.time() - prev

            # Porportional controller.
            # Linear velocity in the x-axis.
            lin = linear_vel(goal_pose)
            vel_msg.linear.x = lin
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            zz=angular_vel(goal_pose)

            average = sum(Av_store) / float(len(Av_store))
            if average > 30:
                zz = zz/average
                vel_msg.linear.x = linear_vel(goal_pose)/4
                vel_msg.angular.z = zz
                velocity_publisher.publish(vel_msg)
                rate.sleep()
            else:

                vel_msg.angular.z = zz
                velocity_publisher.publish(vel_msg)         # Publishing our vel_msg
                rate.sleep()         # Publish at the desired rate.

            Av_store.append(abs(rad_to_deg(zz)))
            Dist_Err=euclidean_distance(goal_pose) #>= distance_tolerance

            if len(Av_store) > 60:           #moving avg.
                Av_store = Av_store[-60:]
            average = sum(Av_store) / float(len(Av_store))
            current_degree = rad_to_deg(yaw)
            if Dbug==1:
                print('Target X: ', goal_pose.transform.translation.x, 'Target Y: ', goal_pose.transform.translation.y)
                print('Current X: ', pose_tf.transform.translation.x, 'Current Y: ', pose_tf.transform.translation.y)
                print("Current Angle (Deg): ",current_degree)            
                print()
                print('Distance error: ',Dist_Err)
                print('Current Angle Error (Deg): ',average)
                print('Exec STEP: ',step, '  Exec STEP Time: ',TIM,' sec.')
                print('Time Elapsed: ',time_elapsed)
                print()

            if (Dist_Err <= distance_tolerance): #and (average <= deg_tolerance):
                flg = True
                break

            if (time_elapsed > Limit)and(average>1): #
                vel_msg.linear.x = 0      # Stopping our robot after the movement is over.
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(1)
                flg = False
                break


        vel_msg.linear.x = 0      # Stopping our robot after the movement is over.
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        #rospy.sleep(3)

        #rospy.spin()     # If we press control + C, the node will stop.

    

def rotation_controller():
#def rotation_controller(x,y,angl): ##will be used for precise alignment to next target
    flg2 = False
    Av_store = []
    deg_tolerance=0.15
    global goal_pose
    while flg2 == False:
        vel_msg = Twist()
        Av_store.append(1)
        prev2 = time.time()
        while True:
            time_elapsed2 = time.time() - prev2

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            zz=angular_vel(goal_pose)
            average = sum(Av_store) / float(len(Av_store))
            if average > 10:
                zz=zz/average
                vel_msg.linear.x = 0
                vel_msg.angular.z = zz
                velocity_publisher.publish(vel_msg)
                rate.sleep()

            vel_msg.angular.z = zz

            velocity_publisher.publish(vel_msg)         # Publishing our vel_msg
            Av_store.append(abs(rad_to_deg(zz)))
            rate.sleep()         # Publish at the desired rate.


            current_degree = rad_to_deg(yaw)
            #if yaw<0: #clockwise
    #            current_degree = 180-current_degree
    #            print("minus: ",current_degree)
            if len(Av_store) > 60:           #moving avg.
                Av_store = Av_store[-60:]
            average = sum(Av_store) / float(len(Av_store))

            print()

            print('Current Angle Error (Deg): ',average)
            print("Current Angle (Deg): ",current_degree)
            print('Time2 Elapsed: ',time_elapsed2)

            if average <= deg_tolerance:
                flg2 = True
                break

                
            if (time_elapsed2 > Limit)and(average>deg_tolerance): #
                vel_msg.linear.x = 0      # Stopping our robot after the movement is over.
                vel_msg.angular.z = 4
                velocity_publisher.publish(vel_msg)
                rospy.sleep(4)
                vel_msg.linear.x = 0      # Stopping our robot after the movement is over.
                vel_msg.angular.z = 0
                velocity_publisher.publish(vel_msg)
                rospy.sleep(1)
                break


                
    vel_msg.linear.x = 0      # Stopping our robot after the movement is over.
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def Auto_SLAM(): #Predefined Solution 1 By giving X-Y coordinates either via a stream or storing in a list or array, then using functions to translate and rotate the bot.
    old_TIM = time.time()
    X_coordinates = [3,3,5,3,3,3,4.8,4.8,4.8,3,3,0,5,3,3,4.2,4.2,5,4.2,4.2,1,1,0,3,3,1,3,3,2,2,3,3,0]
    Y_coordinates = [0,2.5,3,2.5,0,-0.5,-0.5,1,-0.5,-0.5,-2,-2,-2,-2,-3.5,-3.5,-5,-5,-5,-3.5,-3.7,-5.2,-5.2,-5.5,-7,-7,-7,-5.5,-5.5,-4,-4,0,0]
    #------ loop to print Square
    for i in range (0,len(X_coordinates)):                                    #i from 0 to 3
        TIM = time.time() - old_TIM
        move2coord(X_coordinates[i],Y_coordinates[i],1,i+1,round(TIM,2))
        #print(X_coordinates[i],Y_coordinates[i])
        #print(i+1)

rospy.init_node('turtlebot3_controller', anonymous=True) # Creates a node with name 'turtlebot3_controller' and make sure it is a unique node name (using anonymous=True).
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publisher which will publish to the topic
tf_subscriber=rospy.Subscriber('/tf', TFMessage, update_tf)
pose_tf=TransformStamped()

rate = rospy.Rate(200)


Auto_SLAM()
