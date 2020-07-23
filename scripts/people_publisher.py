#!/usr/bin/env python


import rospy
from group_msgs.msg import People, Person
from geometry_msgs.msg import Pose, PoseArray
import tf
import math
from algorithm import SpaceModeling
import copy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


STRIDE = 0.65 # in m

def rotate(px, py, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """

    qx = math.cos(angle) * px - math.sin(angle) * py
    qy = math.sin(angle) * px + math.cos(angle) * py
    return qx, qy

def callback(data):
    group = []
    listener = tf.TransformListener()
    if not data.poses:
        group = []
    else:
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        tx = trans[0]
        ty = trans[1]
        t_quarterion = rot
        (t_roll, t_pitch, t_yaw) = tf.transformations.euler_from_quaternion(t_quarterion)



        for pose in data.poses:
            rospy.loginfo("Person Detected")

            quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

            (px, py) = rotate(pose.position.x, pose.position.y, t_yaw)
            pose_x = px + tx
            pose_y = py + ty
            pose_yaw = yaw + t_yaw

            #pose_person = [pose_x, pose_y, pose_yaw]

            pose_person = [pose.position.x, pose.position.y,yaw]
            #fazer transofmracoa para map FIX!!!!!!
            group.append(pose_person)


    
    aux_group = copy.deepcopy(group)
    groups = [aux_group]
    for gp in groups:
        for p in gp:
            p[0] = p[0] * 100 #algorithm uses cm
            p[1] = p[1] * 100 # m to cm

    app = SpaceModeling(groups)
    pparams,gparams, approaching_poses = app.solve()


    #factor = rospy.get_param("/costmap_node/costmap/social/factor")
    factor = 2
    sx = (float(pparams[0][0])/100)/factor # cm to m
    sy = float(pparams[0][1])/100 # cm to m
    gvar = float(gparams[0]) / 100  # cm to m

    rospy.loginfo(approaching_poses) # in m


    talker(group,sx,sy,gvar, approaching_poses)



def talker(group,sx,sy,gvar, approaching_poses):
    pub = None
    pub = rospy.Publisher('/people', People, queue_size=10)
    p = None

    #while not rospy.is_shutdown():
    p = People()
    p.header.frame_id = "/map"
    p.header.stamp = rospy.Time.now()

    if not group:
        pub.publish(p)
    else:
        for person in group:

            p1 = None
            p1 = Person()
            p1.position.x = person[0]
            p1.position.y = person[1]
            p1.position.z = person[2]
            p1.orientation = person[2]
            p1.sx = sx
            p1.sy = sy
            p.people.append(p1)

        p1 = None
        p1 = Person()
        center = calc_o_space(group)
        p1.position.x = center[0]
        p1.position.y = center[1]
        p1.orientation = math.pi
        p1.sx = gvar
        p1.sy = gvar
        p.people.append(p1)
        pub.publish(p)

    pub_poses = None
    pub_poses = rospy.Publisher('/approaching_poses',PoseArray, queue_size = 10)
    p_pose = None
    p_pose = PoseArray()
    p_pose.header.frame_id = "/map"
    p_pose.header.stamp = rospy.Time.now()
    if not group:
        pub_poses.publish(p_pose)
    else:
        for approaching_pose in approaching_poses: 
        #fazer um for para os varias poses de aproximacao
            p1_pose = None
            p1_pose = Pose()
            p1_pose.position.x = approaching_pose[0] /100 #cm to m
            p1_pose.position.y = approaching_pose [1]/100 # cm to m
            goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, approaching_pose[2])
            p1_pose.orientation.x = goal_quaternion[0]
            p1_pose.orientation.y = goal_quaternion[1]
            p1_pose.orientation.z = goal_quaternion[2]
            p1_pose.orientation.w = goal_quaternion[3]

            p_pose.poses.append(p1_pose)


        pub_poses.publish(p_pose)


#     goal_pose = approaching_pose[0:2]
#     goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, approaching_pose[2])
#     try:
#         rospy.loginfo("Approaching group!")
#         result = movebase_client(goal_pose, goal_quaternion)
#         if result:
#             rospy.loginfo("Goal execution done!")
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")

# def movebase_client(goal_pose, goal_quaternion):

#     client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#     client.wait_for_server()

#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = goal_pose[0]
#     goal.target_pose.pose.position.y = goal_pose[1]
#     goal.target_pose.pose.orientation.x = goal_quaternion[0]
#     goal.target_pose.pose.orientation.y = goal_quaternion[1]
#     goal.target_pose.pose.orientation.z = goal_quaternion[2]
#     goal.target_pose.pose.orientation.w = goal_quaternion[3]

#     client.send_goal(goal)
#     wait = client.wait_for_result()
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#         return client.get_result()

def calc_o_space(persons):
    """Calculates the o-space center of the group given group members pose"""
    c_x = 0
    c_y = 0

    # Group size
    g_size = len(persons)

    for person in persons:
        c_x += person[0] + math.cos(person[2]) * STRIDE
        c_y += person[1] + math.sin(person[2]) * STRIDE

    center = [c_x / g_size, c_y / g_size]

    return center

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/faces",PoseArray,callback)
    rospy.spin()
