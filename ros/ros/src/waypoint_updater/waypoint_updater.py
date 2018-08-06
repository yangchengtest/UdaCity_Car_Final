#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math
import numpy as np
from std_msgs.msg import Int32
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.cur_pose = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_idx = -1
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.cur_pose = msg.pose
        if self.waypoints_2d is not None:
            self.publish()

    def waypoints_cb(self, lane):
        # TODO: Implement
        '''
        if self.base_waypoints is None:
            self.base_waypoints = lane
        '''
        if lane:
            self.base_waypoints = lane
            if not self.waypoints_2d and lane.waypoints is not None:
                self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                     for waypoint in lane.waypoints]
                ### print(len(self.waypoints_2d))
                self.waypoints_tree = KDTree(self.waypoints_2d)
        else:
            rospy.logerr("lane is None")
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        rospy.logwarn("traffic item: %d:", msg.data)
        if self.stopline_wp_idx > -1:
            self.publish()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint_id(self):
        x = self.cur_pose.position.x
        y = self.cur_pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        # check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if (val>0):
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)
        return closest_idx

    def decelerate_waypoints(self,base_waypoints,next_waypoint_index):
        temp = []
        for i,wp in enumerate(base_waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - next_waypoint_index -2 ,0)
            dist = self.distance(base_waypoints, i, stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.0:
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp



    def publish(self):
        next_waypoint_index = self.get_closest_waypoint_id()
        farthest_index = next_waypoint_index + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[next_waypoint_index:farthest_index]
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >=farthest_index):
            result_waypoints = base_waypoints
        else:
            rospy.logwarn("slow down")
            result_waypoints = self.decelerate_waypoints(base_waypoints,next_waypoint_index)

        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = result_waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
