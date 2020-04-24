#!/usr/bin/env python
import rospy
import time
import math

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_msgs.msg import Bool


class PositionController():

  def __init__(self):

    # Allow the simulator to start
    time.sleep(4)
    
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Getting the PID parameters
    stable_gains = rospy.get_param('/position_controller_node/gains/stable/', {'p': 1, 'i': 0.0, 'd': 0.0})
    Kp_s, Ki_s, Kd_s = stable_gains['p'], stable_gains['i'], stable_gains['d']

    # If the speed is set to unstable waypoint
    Kp = Kp_s
    Ki = Ki_s
    Kd = Kd_s

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.pos_x_PID = PID(Kp, Ki, Kd, self.rate)
    self.pos_y_PID = PID(Kp, Ki, Kd, self.rate)
    self.pos_z_PID = PID(Kp, Ki, Kd, self.rate)

    # Get the setpoints
    self.x_setpoint = 0
    self.y_setpoint = 0
    self.z_setpoint = 2.5

    # Create the current output readings
    self.x_pos = 0
    self.y_pos = 0
    self.z_pos = 0

    # Create the subscribers and publishers
    self.vel_set_sub = rospy.Publisher('/uav/input/velocity', Vector3, queue_size=1)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
    self.pos_set_sub = rospy.Subscriber("uav/input/position", Vector3, self.set_pos)
    self.at_goal_pub = rospy.Publisher("uav/sensors/atwaypoint", Bool, queue_size=1)

    # Run the communication node
    self.ControlLoop()


  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(5)

    # Keep track how many loops have happend
    loop_counter = 0

    # While running
    while not rospy.is_shutdown():

        # Use a PID to calculate the velocity you want
        x_proportion = self.pos_x_PID.get_output(self.x_setpoint, self.x_pos)
        y_proportion = self.pos_y_PID.get_output(self.y_setpoint, self.y_pos)
        z_proportion = self.pos_z_PID.get_output(self.z_setpoint, self.z_pos)

        # Initialize the components of the vector
        x_vel = 0
        y_vel = 0
        z_vel = 0

        # Set the velocity based on distance
        x_vel = x_proportion
        y_vel = y_proportion
        z_vel = z_proportion

        # Create and publish the data
        velocity = Vector3(x_vel, y_vel, z_vel)
        self.vel_set_sub.publish(velocity)

        # If we are close to the goal
        dx = self.x_setpoint - self.x_pos
        dy = self.y_setpoint - self.y_pos
        dz = self.z_setpoint - self.z_pos
        distance_to_waypoint = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        msg = Bool()
        if distance_to_waypoint < 0.25:
          msg.data = True
        else:
          msg.data = False
        self.at_goal_pub.publish(msg)

        # Sleep any excess time
        rate.sleep()


  # Call back to get the gps data
  def get_gps(self, msg):
    self.x_pos = msg.pose.position.x
    self.y_pos = msg.pose.position.y
    self.z_pos = msg.pose.position.z


  # Call back to get the position setpoints
  def set_pos(self, msg):
    # If our set point changes reset the PID build up
    check_x = self.x_setpoint != msg.x
    check_y = self.y_setpoint != msg.y
    check_z = self.z_setpoint != msg.z
    if check_x or check_y or check_z:
      self.pos_x_PID.remove_buildup()
      self.pos_y_PID.remove_buildup()
      self.pos_z_PID.remove_buildup()

    self.x_setpoint = msg.x
    self.y_setpoint = msg.y
    self.z_setpoint = msg.z

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('position_controller_node')
  try:
    poscon = PositionController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
