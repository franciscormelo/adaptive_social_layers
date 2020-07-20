#!/usr/bin/env python


import rospy
from people_msgs.msg import People, Person
from geometry_msgs.msg import Pose, PoseArray
import tf
import math
from algorithm import SpaceModeling
import copy

STRIDE = 0.65 # in m
 
def callback(data):
    group = []
    if not data.poses:
        group = []
    else:
        for pose in data.poses:
            rospy.loginfo("Person Detected")

            quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

            pose_person = [pose.position.x, pose.position.y, yaw] #in base_footprint
            #fazer transofmracoa para map FIX!!!!!!
            group.append(pose_person)

    #rospy.loginfo(group)
    aux_group = copy.deepcopy(group)
    groups = [aux_group]
    for gp in groups:
        for p in gp:
            p[0] = p[0] * 100
            p[1] = p[1] * 100

    app = SpaceModeling(groups)
    pparams,gparams = app.solve()


    factor = rospy.get_param("/costmap_node/costmap/social/factor")
    sx = (float(pparams[0][0])/100)/factor
    sy = float(pparams[0][1])/100
    gvar = float(gparams[0]) / 100

    
    rospy.set_param("/costmap_node/costmap/social/varx", sx)
    rospy.set_param("/costmap_node/costmap/social/vary", sy)
    rospy.set_param("/costmap_node/costmap/social/groupvar", gvar)
  
    talker(group)
    
# VER SE O SET PARAM ESTA A FUNCIONAR BEM

def talker(group):
    pub = None
    pub = rospy.Publisher('/people', People, queue_size=10)
    p = None

    #while not rospy.is_shutdown():
    p = People()
    p.header.frame_id = "base_footprint" #devia ser map FIX!!!!!!
    p.header.stamp = rospy.Time.now()

    if not group:
        pub.publish(p)
    else:
        for person in group:
  
            p1 = None
            p1 = Person()
            
        
            #p1.name = "hello"
            p1.position.x = person[0]
            p1.position.y = person[1]
            p1.position.z = 0

            if person[2] > 0:
                p1.velocity.x = 1
            else:
                p1.velocity.x = -1
            

            p1.velocity.y = math.tan(person[2])
            p1.velocity.z = 0
            p1.reliability = 0
            # p1.tagnames = "hello"
            # p1.tags = "hello"
            p.people.append(p1)


        p1 = None
        p1 = Person()
        center = calc_o_space(group)
        p1.position.x = center[0]
        p1.position.y = center[1]
        p.people.append(p1)


        pub.publish(p)


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