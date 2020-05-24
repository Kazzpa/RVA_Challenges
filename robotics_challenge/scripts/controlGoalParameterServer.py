#!/usr/bin/env python


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import sys
import os
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion,PoseStamped, Pose, Point, Vector3,Twist,PointStamped
from std_msgs.msg import Header, ColorRGBA,Bool
from nav_msgs.msg import Path

execute_main = False
class Turtlebot():
    def __init__(self):

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        
        #TOPICS Publishers:
        self.cmd_vel = rospy.Publisher(
            'cmd_vel/tracker', Twist, queue_size=10)
        self.cmd_vel_navi = rospy.Publisher(
            'cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher(
            'visualization_marker', Marker, queue_size=10)
        
        #TOPICS SUBSCRIBER
        self.goal_within_obstacle = rospy.Subscriber(
            "is_goal_within_obstacle", Bool, self.skip_Goal)
        self.path_sub = rospy.Subscriber(
            "path_plan", Path, self.path_recieved)
        self.listener = tf.TransformListener()

        # Parameters
        self.path = None
        self.skipping_Goal = False
        self.path_set = False
        self.skip_goal_max_distance = rospy.get_param('~skip_goal_max_distance',default = 1.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', default=0.10)
        self.linear_turning = rospy.get_param('~linear_turning', default=0.05)
        self.linear_max = rospy.get_param('~linear_max', default=0.50)
        self.angular_max = rospy.get_param('~angular_max', default=0.20)

        #Parametro angle_tolerance en grados
        self.angle_tolerance = rospy.get_param('~angle_tolerance', default=5.0)
 
    
    def command(self, i):
        #rospy.loginfo("Command")

        gx = self.path.poses[i].pose.position.x
        gy = self.path.poses[i].pose.position.y
        goal = PointStamped()
        base_goal = PointStamped()

        goal.header.frame_id = "map"

        goal.header.stamp = rospy.Time()

        goal.point.x = gx
        goal.point.y = gy
        goal.point.z = 0.0
        try:
            base_goal = self.listener.transformPoint('base_link', goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return False, i
        # TODO: put the control law here
        # TODO: with the reactive force to avoid a possible obstacle detected by the laser

        #Publicamos el nodo a llegar en visualization_marker    
        robot.marker_goal(base_goal.point.x,base_goal.point.y)
        # Calculamos la direccion a la que moverse en angulos
        # Solo se pone una coordeanda porque la segunda a restar seria la actual
        # y como es la del robot seria (0,0) por lo que no la ponemos
    
        radians = math.atan2(base_goal.point.y,
                             base_goal.point.x)
        degrees = math.degrees(radians)
        angular = 0.0
        #rospy.loginfo("Distancia x: %f y: %f",
         #             base_goal.point.x, base_goal.point.y)
         # Calculamos la distancia hacia el punto en linea recta
        distance = math.sqrt(math.pow(base_goal.point.x, 2) +
                             math.pow(base_goal.point.y, 2))

        i = self.robot_closer_next_goal(i)
        #rospy.loginfo("Control: Checking skip_goal")
        if self.skipping_Goal:

            if distance < self.skip_goal_max_distance:
                rospy.logwarn("Control: Skipped goal with distance: %.2f", distance)
            else:
                ##Try to 
                rospy.logwarn("Control: Turning manually to avoid obstacles%.2f", distance)
                angular = -self.angular_max * 0.50
                #if degrees < 0:
                #    angular = -1s
                linear = self.linear_turning * 1.10
                self.publish_navi(linear,angular)
            self.skipping_Goal = False
            return False, i
        else:
            #rospy.loginfo("control: unskipped")
            #rospy.loginfo("Distancia: %.2f", distance)
            # Aplicamos movimiento angular en funcion a la direccion la que se encuentre el objetivo
            if degrees > self.angle_tolerance:
                angular = self.angular_max
                linear = self.linear_turning
                #rospy.loginfo("Degrees > %f: %f", self.angle_tolerance, degrees)
            elif degrees < -self.angle_tolerance:
                angular = -self.angular_max
                linear = self.linear_turning
                #rospy.loginfo("Degrees < %f: %f", self.angle_tolerance, degrees)
            else:
                #rospy.loginfo("-%f < Degrees < %f",self.angle_tolerance,self.angle_tolerance)
                linear = 0.9 * distance
                angular = degrees * 0.01 * self.angular_max

            
            # Comprobamos si hemos llegado al destino
            if math.fabs(distance) <= self.goal_tolerance:
                angular = 0.0
                linear = 0
                rospy.loginfo("Robot:Hemos llegado")
                return True,i+1
            #rospy.loginfo("angular %f", angular)
            #rospy.loginfo("linear  %f", linear)
            if math.fabs(linear) > self.linear_max:
                #rospy.logwarn("Limite Linear negativo")
                if linear > self.linear_max:
                    linear = self.linear_max
                else:
                    linear = -self.linear_max
            if math.fabs(angular) > self.angular_max:
                #rospy.logwarn("Limite Angular")
                if angular > self.angular_max:
                    angular = self.angular_max
                else:
                    angular = -self.angular_max
            self.publish(linear, angular)
            return False,i

    def path_recieved(self,path):
        self.path = path
        self.path_set = True
        
    def skip_Goal(self,received):
        self.skipping_Goal = received.data

    def robot_closer_next_goal(self,i):
        success = False
        first_goal_ind = i
        base_goal = PointStamped()
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time()
        goal.point.x = self.path.poses[i].pose.position.x
        goal.point.y = self.path.poses[i].pose.position.y
        goal.point.z = 0.0

        base_goal = PointStamped()
        try:
            base_goal = self.listener.transformPoint('base_link', goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return i
        best_distance = math.sqrt(math.pow(base_goal.point.x, 2) +
                             math.pow(base_goal.point.y, 2))
        last_distance = best_distance
        best_i = i
        while last_distance <= best_distance and i < len(self.path.poses):
            gx = self.path.poses[i].pose.position.x
            gy = self.path.poses[i].pose.position.y
            goal.header.frame_id = "map"

            goal.header.stamp = rospy.Time()

            goal.point.x = gx
            goal.point.y = gy
            goal.point.z = 0.0
            base_goal = PointStamped()
            base_goal = self.listener.transformPoint('base_link', goal)

            last_distance = math.sqrt(math.pow(base_goal.point.x, 2) +
                                math.pow(base_goal.point.y, 2))
            if last_distance <= best_distance:
                best_distance = last_distance
                best_i = i
                i = i+1
        if (i - first_goal_ind)/2 > 1:
            rospy.logwarn("Saltados %.0f pasos del path",(i - first_goal_ind)/2)
        return best_i

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)

    def publish_navi(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
        # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel
        self.cmd_vel_navi.publish(move_cmd)

    def marker_goal(self,goalx,goaly):
        obstacle = Point(goalx, goaly, 0)
        marker = Marker(
            type=Marker.SPHERE,
            id=i,
            lifetime=rospy.Duration(0.10),
            pose=Pose(obstacle, Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.05, 0.05, 0.05),
            header=Header(frame_id='base_link'),
            color=ColorRGBA(0.0, 0.0, 1.0, 0.8))
        #rospy.loginfo("Control: publicado")
        self.marker_publisher.publish(marker)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('robotcontrol', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("Control: To stop TurtleBot CTRL + C")

        robot = Turtlebot()
        # What function to call when you ctrl + c
        rospy.on_shutdown(robot.shutdown)

        # TODO 2: extend it to load more than one goal with base in "path" parameter
        # Como es un parametro compuesto por 2 parametros comprobamos que se haya enviado
        #rospy.loginfo(str(robot.angular_max))

        # tell user how to stop TurtleBot

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)
        i = 0
        rospy.loginfo("Robot iniciado: Comienza bucle")
        # Len/2 bc its an array of x,y coordinates
        goal_reached = False
        goal = True
        while not rospy.is_shutdown() and  not goal_reached:
            if robot.path_set:
                goalx = robot.path.poses[i].pose.position.x
                goaly = robot.path.poses[i].pose.position.y
                if goal:
                    rospy.loginfo("Control: goal actual: %.0f de: %.0f",i,(len(robot.path.poses)))
                    rospy.loginfo("Control:Siguiente goal %f x: %f y: %f",i,goalx,goaly)
                #rospy.loginfo("Loop: %f",i)
                # publish the velocity
                #print("Llamada a command")
                goal,i = robot.command(i)
                if i >= len(robot.path.poses):
                    goal_reached = True
            
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
        rospy.loginfo("Control: Finalizado trayecto")

    except:
        rospy.loginfo("robotcontrol node terminated.")
