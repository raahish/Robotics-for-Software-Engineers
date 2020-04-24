#!/usr/bin/env python
import rospy
import time
import math
import numpy as np

from random import randrange
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


class Hiker():

    def __init__(self):

        # Allow the simulator to start
        time.sleep(1)

        # When this node shutsdown
        rospy.on_shutdown(self.shutdown_sequence)

        # Set the rate
        self.rate = 2.0
        self.dt = 1.0 / self.rate

        # Hiker position
        self.hiker_position = None
        self.map = None
        self.drone_position = [0, 0, 0]

        # Transform
        self.towertransform = TransformStamped()
        self.towertransform.transform.translation.x = float(-128.575)
        self.towertransform.transform.translation.y = float(77.256)
        self.towertransform.transform.translation.z = float(0)

        quat = quaternion_from_euler(float(0), float(0), float(+0.523599))
        self.towertransform.transform.rotation.x = quat[0]
        self.towertransform.transform.rotation.y = quat[1]
        self.towertransform.transform.rotation.z = quat[2]
        self.towertransform.transform.rotation.w = quat[3]

        self.towertransform.header.frame_id = 'tower'
        self.towertransform.child_frame_id = 'world'

        # Init map details
        self.width = 0
        self.height = 0
        self.origin_x = 0
        self.origin_y = 0
        self.width = 0

        # Create the subscribers and publishers
        self.pos_pub = rospy.Publisher("/tower/hiker/position", Vector3, queue_size=1)
        self.true_pub = rospy.Publisher("/hiker/exact/position", Vector3, queue_size=1)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
        self.drone_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
        # Run the communication node
        self.ControlLoop()


    # Call back to get the gps data
    def get_gps(self, msg):
        self.drone_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    # Map callback
    def get_map(self, msg):

        # Get the map width and height
        self.width = msg.info.width
        self.height = msg.info.height

        # Get the drone position
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Get the map
        self.map = np.reshape(msg.data, (self.width, self.height))

        # Set the hikers position
        hiker_pos_found = False
        while hiker_pos_found == False:
            hiker_rand_pos = randrange(4)
            if hiker_rand_pos == 0:
                self.hiker_position = np.array([int(self.width - 1), int(self.height - 1)])
            elif hiker_rand_pos == 1:
                self.hiker_position = np.array([int(1), int(1)])
            elif hiker_rand_pos == 2:
                self.hiker_position = np.array([int(1), int(self.height - 1)])
            elif hiker_rand_pos == 3:
                self.hiker_position = np.array([int(self.width - 1), int(1)])
            else:
                self.hiker_position = np.array([int(self.width + self.origin_x), int(self.height + self.origin_y)])

            if self.map[self.hiker_position[0]][self.hiker_position[0]] == 0:
                hiker_pos_found = True

        # Reset plan
        self.have_plan = False

    # This is the main loop of this class
    def ControlLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # Position movements
        movements = np.array([[1, 1], [1, 0], [1, -1], [0, 1], [0, 0], [0, -1], [-1, 1], [-1, 0], [-1, -1]])

        # Create a counter which lets everything set up
        setup_counter = 0

        # While running
        while not rospy.is_shutdown():

            # Check we have a map
            if self.map is not None:

                # We have not moved yet
                moved = False

                # Move the person for 10 cycles before starting
                if setup_counter < 10:
                    setup_counter += 1

                    # Move the hiker
                    while not moved:
                        # Create a random number
                        action = randrange(movements.shape[0])
                        # Compute where the next position would be
                        position = self.hiker_position + movements[action]
                        # If the next position is inside the map and in free space
                        if self.insideMapAndOpen(position):
                            # Move the hiker
                            self.hiker_position = position
                            # We have moved
                            moved = True
                    # Skip the rest of the loop
                    rate.sleep()
                    continue

                # Compute the distance between the hiker and the drone
                dx = self.drone_position[0] - (self.hiker_position[0] + self.origin_x)
                dy = self.drone_position[1] - (self.hiker_position[1] + self.origin_y)
                dz = self.drone_position[2] - 0
                drone_hiker_distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2))

                # Hiker will move if the drone is not within 5m of the drone
                if drone_hiker_distance > 5:
                    # Move the hiker
                    while not moved:
                        # Create a random number
                        action = randrange(movements.shape[0])
                        # Compute where the next position would be
                        position = self.hiker_position + movements[action]
                        # If the next position is inside the map and in free space
                        if self.insideMapAndOpen(position):
                            # Move the hiker
                            self.hiker_position = position
                            # We have moved
                            moved = True

                # If the drone is within 5m of the drone publish the true position
                if drone_hiker_distance < 5:
                    # Send the world co-ordinates of the hiker
                    msg = Vector3(self.hiker_position[0] + self.origin_x, self.hiker_position[1] + self.origin_y, 0)
                    self.true_pub.publish(msg)

                # If the drone is above 20m publish the tower position of the human
                if self.drone_position[2] > 20:
                    msg = PointStamped()
                    msg.point.x = self.hiker_position[0] + self.origin_x
                    msg.point.y = self.hiker_position[1] + self.origin_y
                    msg.point.z = 0
                    transformed_point = do_transform_point(msg, self.towertransform)
                    msg = Vector3(transformed_point.point.x, transformed_point.point.y, 0)
                    self.pos_pub.publish(msg)

            # Sleep any excess time
            rate.sleep()

    def insideMapAndOpen(self, pos):

        # Inside the map
        if 0 <= pos[0] < self.width and 0 <= pos[1] < self.height:
            # If the cell is free
            if self.map[pos[0], pos[1]] == 0:
                return True
        
        # Otherwise 
        return False

    # Called on ROS shutdown
    def shutdown_sequence(self):
        rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


if __name__ == '__main__':
    rospy.init_node('hiker_node')
    try:
        lostHiker = Hiker()
    except rospy.ROSInterruptException:
        pass

