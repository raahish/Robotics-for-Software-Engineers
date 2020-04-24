#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import math


class VelocityController():

  def __init__(self):

    # Allow the simulator to start
    time.sleep(4)

    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Getting the PID parameters
    gains = rospy.get_param('/velocity_controller_node/gains', {'p_xy': 1, 'i_xy': 0.0, 'd_xy': 0.0, 'p_z': 1, 'i_z': 0.0, 'd_z': 0.0})
    Kp_xy, Ki_xy, Kd_xy = gains['p_xy'], gains['i_xy'], gains['d_xy']
    Kp_z, Ki_z, Kd_z = gains['p_z'], gains['i_z'], gains['d_z']

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p_xy - " + str(Kp_xy))
    rospy.loginfo(str(rospy.get_name()) + ": i_xy - " + str(Ki_xy))
    rospy.loginfo(str(rospy.get_name()) + ": d_xy - " + str(Kd_xy))
    rospy.loginfo(str(rospy.get_name()) + ": p_z - " + str(Kp_z))
    rospy.loginfo(str(rospy.get_name()) + ": i_z - " + str(Ki_z))
    rospy.loginfo(str(rospy.get_name()) + ": d_z - " + str(Kd_z))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.vel_x_PID = PID(Kp_xy, Ki_xy, Kd_xy, self.rate)
    self.vel_y_PID = PID(Kp_xy, Ki_xy, Kd_xy, self.rate)
    self.vel_z_PID = PID(Kp_z, Ki_z, Kd_z, self.rate)
    
    # Get the setpoints
    self.x_setpoint = 0
    self.y_setpoint = 0
    self.z_setpoint = 0

    # Create the current output readings
    self.x_vel = 0
    self.y_vel = 0
    self.z_vel = 0  

    # Reading the yaw
    self.yaw_reading = 0
    self.yaw_setpoint = 0

    # Create the subscribers and publishers
    self.vel_read_sub = rospy.Subscriber("/uav/sensors/velocity", TwistStamped, self.get_vel)
    self.vel_set_sub = rospy.Subscriber('/uav/input/velocity', Vector3 , self.set_vel)
    self.att_sub = rospy.Subscriber('/uav/sensors/attitude', Vector3, self.euler_angle_callback)
    self.yaw_sub = rospy.Subscriber('/uav/input/yaw', Float64, self.yaw_callback)
    self.att_pub = rospy.Publisher("/uav/input/attitude", Vector3, queue_size=1)
    self.thrust_pub = rospy.Publisher('/uav/input/thrust', Float64, queue_size=1)

    # Run the communication node
    self.ControlLoop()

  # Callback to get the yaw reading
  def euler_angle_callback(self, msg):
    self.yaw_reading = msg.z

  # Callback to set the yaw
  def yaw_callback(self, msg):
    self.yaw_setpoint = msg.data

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(10)

    # Keep track how many loops have happend
    loop_counter = 0

    # Data we will be publishing
    z_output = Float64()
    z_output.data = 0

    # While running
    while not rospy.is_shutdown():

      # Use a PID to calculate the angle you want to hold and thrust you want
      x_output_w = self.vel_x_PID.get_output(self.x_setpoint, self.x_vel)
      y_output_w = self.vel_y_PID.get_output(self.y_setpoint, self.y_vel)
      z_output.data = self.vel_z_PID.get_output(self.z_setpoint, self.z_vel) + 9.8

      # Rotation from a world frame to drone frame
      x_output = - math.cos(self.yaw_reading) * x_output_w - math.sin(self.yaw_reading) * y_output_w
      y_output = math.sin(self.yaw_reading) * x_output_w - math.cos(self.yaw_reading) * y_output_w

      # Create and publish the data (0 yaw)
      attitude = Vector3(x_output, y_output, self.yaw_setpoint)
      self.att_pub.publish(attitude) 
      self.thrust_pub.publish(z_output)

      # Limit the thrust to the drone
      if z_output.data < 0:
        z_output.data = 0
      if z_output.data > 20:
        z_output.data = 20

      # Sleep any excess time
      rate.sleep()


  # Call back to get the velocity data
  def get_vel(self, vel_msg):
    self.x_vel = vel_msg.twist.linear.x
    self.y_vel = vel_msg.twist.linear.y
    self.z_vel = vel_msg.twist.linear.z


  # Call back to get the velocity setpoints
  def set_vel(self, vel_msg):
    self.x_setpoint = vel_msg.x
    self.y_setpoint = vel_msg.y
    self.z_setpoint = vel_msg.z


	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('velocity_controller_node')
  try:
    velcon = VelocityController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()