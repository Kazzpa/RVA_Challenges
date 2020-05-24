#!/usr/bin/env python

""" A ROS planning node that subscribes to a costmap
  and generates a path by using the Djikstra algorithm"""

import sys
import math
import rospy
import tf
import yaml
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid, Path
from dijkstra import Dijkstra


class Planner:
    def __init__(self):
        # As a first possibility, we will search for the init and the goal
        # at the parameter server

        rospy.loginfo("In the planner constructor")

        if rospy.has_param('~goal_path'):
            self.goal_path = rospy.get_param(
                '~goal_path', default="goal_path.yaml")
        if rospy.has_param('~init'):
            self.initx = rospy.get_param('~init/x')
            self.inity = rospy.get_param('~init/y')

        if rospy.has_param('~goal'):
            self.goalx = rospy.get_param('~goal/x')
            self.goaly = rospy.get_param('~goal/y')
        rospy.loginfo("Here")
        print('Init (%f, %f). Goal (%f,%f): ' %
              (self.initx, self.inity, self.goalx, self.goaly))
        self.costmap_topic = rospy.get_param(
            '~costmap_topic', default="costmap_2d/costmap/costmap")
        self.marker_pub = rospy.Publisher('path', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('path_plan', Path, queue_size=10)
        rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.map_callback)
        rospy.Subscriber('goal_pose', OccupancyGrid, self.map_callback)
        self.listener = tf.TransformListener()
        self.init = False  # This flag would be raised in the map callback

    def map_callback(self, map):
        self.map = map
        if self.init == False:
            self.init = True
            self.path = self.calculate_path(
                self.initx, self.inity, self.goalx, self.goaly)
            print "llamada a enviar topic path_plan"
            self.publish_path_marker(self.path)
            self.publish_path_msg(self.path)
            self.saveAsYaml(self.path)
            # TODO: 2 add the YAML exporting utility. Hint: use the pyyaml module
            # More information: https://stackabuse.com/reading-and-writing-yaml-to-a-file-in-python/
            # Beware of the header (timestamp and frame_id)

    def calculate_path(self, ix, iy, gx, gy):
        self.dijkstra = Dijkstra(self.map)
        return self.dijkstra.planning(ix, iy, gx, gy)

    def publish_path_msg(self, path):
        print("Planner:Publicando path")
        x, y = path
        x = x[::-1]
        y = y[::-1]
        my_path = Path()
        my_path.header.frame_id = "map"
        my_path.header.stamp = rospy.get_rostime()
        size = range(len(path[0]))
        poses = []
        for i in size:
            p_stamp = PoseStamped()
            p_stamp.pose.position.x = x[i]
            p_stamp.pose.position.y = y[i]
            p_stamp.pose.position.z = 0
            p_stamp.header.frame_id = "map"
            p_stamp.header.stamp = rospy.get_rostime()
            poses.append(p_stamp)
        my_path.poses = poses
        self.path_pub.publish(my_path)

    def publish_path_marker(self, path):
        # TODO: 1 complete the publish marker method by publishing a LINE_STRIP marker
        # Beware of the header (timestamp and frame_id)
        rospy.loginfo('Called publish path marker')
        x, y = path
        marker = Marker(
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            id=1,
            lifetime=rospy.Duration(100000),
            header=Header(frame_id='map'),
            color=ColorRGBA(1.0, 1.0, 1.0, 1.8))
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        size = range(len(path[0]))
        for i in size:
            p = Point(x=x[i], y=y[i], z=0)
            marker.points.append(p)
            aux = "x:" + str(x[i]) + " y:" + str(y[i])
        self.marker_pub.publish(marker)
        rospy.loginfo("Publicado marcador")

    def saveAsYaml(self, path):
        goalsList = {}
        x, y = path
        x = x[::-1]
        y = y[::-1]
        x_str = "x"
        y_str = "y"
        size = range(len(path[0]))
        for i in size:
            aux = "goal"+str(i)
            aux_dic = {}
            aux_dic[x_str] = x[i]
            aux_dic[y_str] = y[i]
            goalsList[aux] = aux_dic
        goalsYaml = {}
        goalsYaml["path"] = goalsList
        with open(self.goal_path, 'w') as file:
            documents = yaml.dump(goalsYaml, file)
        print "Guardado archivo en " + self.goal_path


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('planning', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("Starting planning node. Waiting for valid map")

        planner = Planner()

        # The planning node will wait for new goals and the map at this rate
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            r.sleep()

    except:
        rospy.loginfo("Planning node terminated.")
