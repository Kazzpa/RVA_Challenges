#! /usr/bin/python
# Copyright (c) 2013 Mak Nazecic-Andrlon
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from __future__ import division

from pyorca import Agent, get_avoidance_velocity, orca, normalized, perp
from numpy import array, rint, linspace, pi, cos, sin, mod, isnan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf

import rospy
import itertools
import random
import math


class ros_orca():
    def __init__(self):

        # Publishers
        # Publish to gazebo controller
        self.cmd_vel = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher(
            'visualization_marker', Marker, queue_size=10)
        self.goal_near_obstacle= rospy.Publisher(
            'is_goal_within_obstacle', Bool, queue_size=10)

        # Suscriptores
        self.cmd_vel_subscriber = rospy.Subscriber(
            "/cmd_vel/tracker", Twist, self.callback_cmd_vel)
        self.laser_scan = rospy.Subscriber(
            "/scan", LaserScan, self.callback_scan)
        # Parameters
        self.linear_threshold = rospy.get_param('~linear_threshold', default=0.01)
        self.linear_max = rospy.get_param('~linear_max', default=0.40)
        self.radius = rospy.get_param('~agent_radius', default=0.40)
        self.delta_t = rospy.get_param('~delta_time', default=0.10)
        self.max_distance_obstacle = rospy.get_param('~max_distance_obstacle', default=2.0)
        self.v_orca = None
        self.linear = None
        self.angular = None
        self.laser_init = False

        self.listener = tf.TransformListener()

    # Invoked by /cmd_vel/tracker, transform the speed into a vector
    #  as v_orca whic is used by orca algorithm
    def callback_cmd_vel(self, data):
        #rospy.loginfo("Orca:Recibido datos de controlGoal")
        self.linear = data.linear.x
        if self.linear < self.linear_threshold:
            self.angular = data.angular.z
        else:
            self.angular = data.angular.z * self.delta_t
        self.v_orca = (self.linear * cos(self.angular), self.linear * sin(self.angular))


    # Publish the calculated veocity to turtlebot
    def publish(self,lin_vel,ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel
        #rospy.loginfo("Orca:Movimiento enviado")
        self.cmd_vel.publish(move_cmd)

    #Analyzes the obstacles retrieved from LaserScan, filters the ones 
    #who are farther than max_distance_obstacle, and transform v_orca 
    #in a new vector to avoid the obstacles. If orca can't transform 
    #the direction a new one, it will raise an exception and True 
    #will be send in /is_goal_within_obstacle topic
    def callback_scan(self, data):
        if self.v_orca != None:
            # if linear speed is greater than the threshold then it
            # isn't considered as a turning only movement, so its
            # considered for avoiding obstacle by orca
            if self.linear > self.linear_threshold:

                #rospy.loginfo("Orca: Calculando para evitar obstaculos")
                self.laser_scan = data
                self.laser_init = True
                skip_goal = False
                #rospy.loginfo("Orca:Laser received " + str(len(data.ranges)))
                agents = []
                agents.append(Agent((0., 0.), self.v_orca,self.radius, self.linear_max, self.v_orca))
                for i in range(1, len(self.laser_scan.ranges), 40):
                    angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
                    d = self.laser_scan.ranges[i]
                    if not isnan(d) and d < self.max_distance_obstacle:
                        #rospy.loginfo(str(angle))
                        obstacle = Point(d * cos(angle), d * sin(angle), 0)
                        marker = Marker(
                            type=Marker.SPHERE,
                            id=i,
                            lifetime=rospy.Duration(self.delta_t),
                            pose=Pose(obstacle, Quaternion(0, 0, 0, 1)),
                            scale=Vector3(self.radius, self.radius, self.radius),
                            header=Header(frame_id='base_link'),
                            color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
                        self.marker_publisher.publish(marker)
                        pos = obstacle.x, obstacle.y
                        agents.append(Agent(pos, (0., 0.), 0.01, 0.1, (0., 0.,)))
                try:
                    v = self.orca_apply(agents)
                    linear_vel = math.sqrt(v[0]**2 + v[1]**2)
                    ang_vel = math.atan2(v[1],v[0]) / self.delta_t
                except:
                    rospy.logerr("Orca: InfeasibleError")
                    skip_goal = True
                    ang_vel = 0
                    linear_vel = 0
                self.publish(linear_vel,ang_vel)
                self.goal_near_obstacle.publish(Bool(data=skip_goal))
                #rospy.loginfo("Orca: Evitando obstaculos")
            else:
                #rospy.logwarn("Orca: Desplazado sin evitar obstaculos")
                self.publish(self.linear,self.angular)

    #Calls orca algorithm to resolve the direction which the robot
    # must move to avoid obstacle.
    def orca_apply(self, agents):
        #Todo: catch InfeasibleError
        dt = self.delta_t
        tau = 1
        new_vels = [None] * len(agents)
        all_lines = [[]] * len(agents)
        for i, agent in enumerate(agents):
            candidates = agents[:i] + agents[i + 1:]
            # print(candidates)
            new_vels[i], all_lines[i] = orca(agent, candidates, tau, dt)
            # print(i, agent.velocity)

        return new_vels[0]


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('orca', anonymous=False)

    
        orca_instance = ros_orca()

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

        # What function to call when you ctrl + c
        # rospy.on_shutdown(robot.shutdown)

        while not rospy.is_shutdown():
            #rospy.loginfo("Orca: Loop")
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
    except:
        rospy.loginfo("robotcontrol node terminated.")
