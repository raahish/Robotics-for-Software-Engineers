#!/usr/bin/env python
import rospy
import copy

import numpy as np

from astar_class import AStarPlanner
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid


# Create a class which we will use to take keyboard commands and convert them to a position
class PathPlanner():
  # On node initialization
  def __init__(self):

    # Init map variables
    self.map = []
    self.width = -1
    self.height = -1
    self.origin_x = 0
    self.origin_y = 0

    # Init the drone and goal position
    self.drone_position = []
    self.goal_position = []

    # Init variables used by the trajectory
    self.requested_height = 25.0
    self.cancel = False

    # Init variables used to control the main loop
    self.have_plan = False
    self.at_waypoint = False
    self.goal_changed = False

    # Create the subscriber nodes
    self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
    self.requested_position = rospy.Subscriber("/uav/input/goal", Vector3, self.get_goal)
    self.atwaypoint_sub = rospy.Subscriber('uav/sensors/atwaypoint', Bool, self.get_at_waypoint, queue_size = 1)
    self.cancel_traj = rospy.Subscriber('cancel/trajectory', Empty, self.get_cancel, queue_size = 1)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)

    # Create the publishers
    self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
    self.trajectory_pub = rospy.Publisher('/uav/trajectory', Int32MultiArray, queue_size=1)

    # Call the mainloop of our class
    self.mainloop()

  # Callback: Get the GPS data
  def get_gps(self, msg):
    self.drone_position = [int(round(msg.pose.position.x - self.origin_x, 0)), int(round(msg.pose.position.y- self.origin_y, 0))]

  # Callback: Check if the current trajectory has been canceled
  def get_cancel(self, msg):
    self.cancel = True

  # Callback: Get the map
  def get_map(self, msg):
    # Get the map width and height
    self.width = msg.info.width
    self.height = msg.info.height
    # Get the drone position
    self.origin_x = msg.info.origin.position.x
    self.origin_y = msg.info.origin.position.y
    # Get the map
    self.map = np.reshape(msg.data, (self.width, self.height))

  # Callback: Check if the drone has reached the current waypoint
  def get_at_waypoint(self, msg):
    self.at_waypoint = msg.data

  # Callback: Get the goal
  def get_goal(self, msg):
    # if len(self.goal_position) == 0:
    rospy.loginfo(str(rospy.get_name()) + ": Received Goal {}".format((int(round(msg.x, 0)), int(round(msg.y, 0)))))
    # Get the goal position
    x = int(round(msg.x, 0) - self.origin_x)
    y = int(round(msg.y, 0) - self.origin_y)
    self.requested_height = int(round(msg.z, 0))
    # Get the drone position
    self.goal_position = [x, y]
    self.goal_changed = True
    self.cancel = False

  # The main loop
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(3)

    # Checks if the plan has been started
    sent_position = False
    trajectory = None
    current_trajectory = None

    # Create the trajectory publish message
    p_traj = Int32MultiArray()
    current_waypoint = Vector3()

    # While ROS is still running
    while not rospy.is_shutdown():

      # If the goal was cancelled
      if self.cancel:
        p_traj.data = []
        p_traj.layout.data_offset = 0
        self.trajectory_pub.publish(p_traj)
        current_trajectory = None
        trajectory = None
        self.goal_changed = False

      # If we have all the data we need to compute a trajectory
      if (len(self.map) != 0) and (len(self.drone_position) == 2) and (len(self.goal_position) == 2) and (self.goal_changed == True):
        # Set that we have tried the current goal
        self.goal_changed = False
        rospy.loginfo(str(rospy.get_name()) + ": Planning trajectory")
        # Try create a trajectory
      # try:
        rospy.loginfo(str(rospy.get_name()) + ": Trying..")
        astar = AStarPlanner(safe_distance=1)
        trajectory = astar.plan(self.map, self.drone_position, self.goal_position)
      # except:
      #   rospy.loginfo(str(rospy.get_name()) + ": Failed")
      #   trajectory = None
        # If we have a trajectory
        if trajectory is not None and len(trajectory)!=0:
          trajectory = np.array(trajectory)
          self.have_plan = True
          trajectory[:, 0] = trajectory[:, 0] + self.origin_x
          trajectory[:, 1] = trajectory[:, 1] + self.origin_y
          rospy.loginfo(str(rospy.get_name()) + ": Executing trajectory")   
          rospy.loginfo(str(rospy.get_name()) + ": " + str(trajectory)) 
        else:
          rospy.loginfo(str(rospy.get_name()) + ": Trajectory not found, try another goal")  

      # If we have a plan save it to current_trajectory
      if trajectory is not None and len(trajectory)!=0:
        current_trajectory = copy.deepcopy(trajectory)
        # Set trajectory to None so we only work with the current trajectory
        trajectory = None

        # Publish the trajectory
        if len(p_traj.data) != len(np.reshape(current_trajectory,-1)):
          p_traj.data = np.reshape(current_trajectory,-1)
          p_traj.layout.data_offset = self.requested_height
          self.trajectory_pub.publish(p_traj)

      if current_trajectory is not None:
        # Publish the current waypoint
        if self.at_waypoint == False or sent_position == False or np.shape(current_trajectory)[0] < 0:
          msg = Vector3()
          msg.x = current_trajectory[0][0]
          msg.y = current_trajectory[0][1]
          msg.z = self.requested_height
          self.position_pub.publish(msg)
          sent_position = True
        else:
          current_trajectory = current_trajectory[1:]
          sent_position = False

        # If we are done wait for next goal
        if np.shape(current_trajectory)[0] <= 0 and self.at_waypoint:
          self.goal_position = []
          current_trajectory = None
          continue

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('path_planning_node')
  try:
    pp = PathPlanner()
  except rospy.ROSInterruptException:
    pass