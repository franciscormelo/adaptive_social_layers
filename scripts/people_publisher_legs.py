#!/usr/bin/env python

import rospy
from group_msgs.msg import People, Person, Groups
from people_msgs.msg import People as Ppl
from geometry_msgs.msg import Pose, PoseArray
import tf
import math
from algorithm import SpaceModeling
import copy

import actionlib

import matlab.engine
eng = matlab.engine.start_matlab()
eng.cd(r'/home/flash/catkin_ws/src/adaptive_social_layers/scripts', nargout=0)

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
        rospy.init_node('PeoplePublisherLegs', anonymous=True)
        
        rospy.Subscriber("/people_laser",Ppl,self.callback,queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.pose_received = False

        self.data = None
        self.pub = rospy.Publisher('/people', People, queue_size=1)
        self.pubg = rospy.Publisher('/groups', Groups, queue_size=1)

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
        print(data)

        if not data.people:
            groups = []
        else:
            for pose in data.people:
                print(pose)
                #!!!! VERY IMPORTANT -> People measurements are given in odom frame

                rospy.loginfo("Person Detected")

                # Compute orientation from velocity
                yaw = math.atan2(pose.velocity.y,pose.velocity.x)
                
                pose_person = [pose.position.x * 100, pose.position.y * 100,yaw]
                persons.append(pose_person)

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
            p.header.frame_id = self.data.header.frame_id
            p.header.stamp = rospy.Time.now()

            g = Groups()
            g.header.frame_id = self.data.header.frame_id
            g.header.stamp = rospy.Time.now()
            
            for idx,group in enumerate(groups):
                aux_p = People()
                aux_p.header.frame_id = self.data.header.frame_id
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
            p.header.frame_id = self.data.header.frame_id
            p.header.stamp = rospy.Time.now()
            self.pub.publish(p)

            g = Groups()
            g.header.frame_id = self.data.header.frame_id
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