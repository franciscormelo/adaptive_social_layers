#!/usr/bin/env python

import matlab.engine
eng = matlab.engine.start_matlab()
eng.cd(r'/home/flash/catkin_ws/src/adaptive_social_layers/scripts', nargout=0)

import rospy
from group_msgs.msg import People, Person, Groups
from geometry_msgs.msg import Pose, PoseArray
import tf
import math
from algorithm import SpaceModeling
import copy
from visualization_msgs.msg import Marker

import actionlib



STRIDE = 65 # in cm
MDL = 8000

# Relation between personal frontal space and back space
BACK_FACTOR = 1.3

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

class PeoplePublisher():
    """
    """
    def __init__(self):
        """
        """
        rospy.init_node('PeoplePublisher', anonymous=True)
        
        rospy.Subscriber("/faces",PoseArray,self.callback,queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.pose_received = False

        self.data = None
        self.pub = rospy.Publisher('/people', People, queue_size=1)
        self.pubg = rospy.Publisher('/groups', Groups, queue_size=1)

        self.pubd = rospy.Publisher('/people_detections', PoseArray, queue_size=1)

    def callback(self,data):
        """
        """
        
        self.data = data
        self.pose_received = True
        

    def publish(self):
        """
        """
        
        data = self.data
        groups = []
        group = []

        persons = []

        ap_points = PoseArray()
        ap_points.header.frame_id = "/base_footprint"
        ap_points.header.stamp = rospy.Time.now()

        if not data.poses:
            groups = []
        else:
            for pose in data.poses:

                rospy.loginfo("Person Detected")


                
                quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


                ######################
                ap_pose = Pose()
                ap_pose.position.x = pose.position.x
                ap_pose.position.y = pose.position.y
                ap_pose.position.z = 0.1

                ap_pose.orientation.x = quaternion[0]
                ap_pose.orientation.y = quaternion[1]
                ap_pose.orientation.z = quaternion[2]
                ap_pose.orientation.w = quaternion[3]
                
                ap_points.poses.append(ap_pose)
                #########################
                (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
                print(yaw)
                pose_person = [pose.position.x * 100, pose.position.y * 100,yaw]
                persons.append(pose_person)
            self.pubd.publish(ap_points)

        # Run GCFF gcff.m Matlab function      
        if persons:
            groups = eng.gcff(MDL,STRIDE, matlab.double(persons))
  
        if groups:
            app = SpaceModeling(groups) # Space modeling works in cm
            pparams,gparams = app.solve()

            ####
            # Inserir aqui parametros adaptados com obstaculos
            ####

            p = People()
            p.header.frame_id = "/base_footprint"
            p.header.stamp = rospy.Time.now()

            g = Groups()
            g.header.frame_id = "/base_footprint"
            g.header.stamp = rospy.Time.now()
            
            for idx,group in enumerate(groups):
                aux_p = People()
                aux_p.header.frame_id = "/base_footprint"
                aux_p.header.stamp = rospy.Time.now()

                sx = (float(pparams[idx][0])/100) # cm to m
                sy = float(pparams[idx][1])/100 # cm to m
                gvarx = float(gparams[idx][0]) / 100  # cm to m
                gvary = float(gparams[idx][1]) / 100  # cm to m
                

    
                ############## FIXED
                #sx = 0.9
                #sy = 0.9
                #########################
                for person in group:

                    p1 = Person()
                    p1.position.x = person[0] / 100 # cm to m
                    p1.position.y = person[1] / 100 # cm to m
                    p1.orientation = person[2]
                    p1.sx = sx 
                    p1.sy = sy

                    p1.sx_back = p1.sx / BACK_FACTOR
                    p1.ospace = False
                    p.people.append(p1)

                    
                    aux_p.people.append(p1)
                
                # Only represent o space for  +2 individuals
                if len(group) > 1:
                    p1 = Person()
                    center = calc_o_space(group)
                    p1.position.x = center[0] / 100 # cm to m
                    p1.position.y = center[1] / 100 # cm to m
                    p1.orientation = math.pi
                    p1.sx = gvarx
                    p1.sy = gvary
                    p1.ospace = True
                    p.people.append(p1)

                    aux_p.people.append(p1)

                g.groups.append(aux_p)

            self.pub.publish(p)
            
            self.pubg.publish(g)

        else:
            p = People()
            p.header.frame_id = "/base_footprint"
            p.header.stamp = rospy.Time.now()
            self.pub.publish(p)

            g = Groups()
            g.header.frame_id = "/base_footprint"
            g.header.stamp = rospy.Time.now()
            self.pubg.publish(g)

    def run_behavior(self):
        while not rospy.is_shutdown():
            if self.pose_received:
                
                self.pose_received = False
                self.publish()
               
if __name__ == '__main__':
    people_publisher = PeoplePublisher()
    people_publisher.run_behavior()
    eng.quit()