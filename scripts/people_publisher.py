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

import actionlib



STRIDE = 0.65 # in m

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
    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        
        rospy.Subscriber("/faces",PoseArray,self.callback,queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.pose_received = False

        self.data = None
        self.pub = rospy.Publisher('/people', People, queue_size=1)

    def callback(self,data):
        
        self.data = data
        self.pose_received = True
        

    def publish(self):
        
        data = self.data
        groups = []
        group = []

        persons = []

        if not data.poses:
            groups = []
        else:
            for ct, pose in enumerate(data.poses):

                rospy.loginfo("Person Detected")
                
                quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

                pose_person = [pose.position.x * 100, pose.position.y * 100,yaw]
                pose_persons_idx = [ct + 1 ,pose.position.x * 100, pose.position.y * 100,yaw,]
                persons.append(pose_person)

        # Run GCFF gcff.m Matlab function      
        mdl = 8000
        stride = 65
        if persons:
            groups = eng.gcff(mdl,stride, matlab.double(persons))
  
        if groups:
            app = SpaceModeling(groups)
            pparams,gparams= app.solve()

            p = People()
            p.header.frame_id = "/base_footprint"
            p.header.stamp = rospy.Time.now()
            
            for idx,group in enumerate(groups):

                sx = (float(pparams[idx][0])/100)/BACK_FACTOR # cm to m
                sy = float(pparams[idx][1])/100 # cm to m
                gvarx = float(gparams[idx][0]) / 100  # cm to m
                gvary = float(gparams[idx][1]) / 100  # cm to m
                

    
                for person in group:

                    p1 = Person()
                    p1.position.x = person[0] / 100 # cm to m
                    p1.position.y = person[1] / 100 # cm to m
                    p1.orientation = person[2]
                    p1.sx = sx
                    p1.sy = sy
                    p1.ospace = False
    
                    p.people.append(p1)

                
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


                
            self.pub.publish(p)

 

        else:
            p = People()
            p.header.frame_id = "/base_footprint"
            p.header.stamp = rospy.Time.now()
            self.pub.publish(p)

    def run_behavior(self):
        while not rospy.is_shutdown():
            if self.pose_received:
                
                self.pose_received = False
                self.publish()

if __name__ == '__main__':

 
    people_publisher = PeoplePublisher()
    people_publisher.run_behavior()
    eng.quit()