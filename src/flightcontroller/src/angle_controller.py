#!/usr/bin/env python
import rospy
import time

from geometry_msgs.msg import Vector3
from mav_msgs.msg import RateThrust
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from pid_class import PID
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class AngleController():

  def __init__(self):

    # Allow the simulator to start
    time.sleep(4)

    # Run the shutdown sequence on shutdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Getting the PID parameters
    gains = rospy.get_param('/angle_controller_node/gains', {'p': 0.1, 'i': 0, 'd': 0})
    Kp, Ki, Kd = gains['p'], gains['i'], gains['d']

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.rollPID = PID(Kp, Ki, Kd, self.rate)
    self.pitchPID = PID(Kp, Ki, Kd, self.rate)
    self.yawPID = PID(Kp, Ki, Kd, self.rate)

    # Create the setpoints
    self.roll_setpoint = 0
    self.pitch_setpoint = 0
    self.yaw_setpoint = -1.57
    self.thrust_setpoint = 10

    # Create the current output readings
    self.roll_reading = 0
    self.pitch_reading = 0
    self.yaw_reading = 0

    # Check if the drone has been armed
    self.armed = False

    # Publishers and Subscribers
    self.rate_pub = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)
    self.imu_sub = rospy.Subscriber("/uav/sensors/attitude", Vector3, self.euler_angle_callback)
    self.att_sub = rospy.Subscriber("/uav/input/attitude", Vector3, self.attitude_set_callback)
    self.thrust_sub = rospy.Subscriber("/uav/input/thrust", Float64, self.thrust_callback)

    # Run the communication node
    self.ControlLoop()

  # This is the callback for the yaw subscriber
  def set_yaw_output(self, msg):
    # rospy.loginfo("Received yaw" + str(msg.data))
    self.yaw_output = msg.data
    
  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(20)

    # Keep track how many loops have happend
    loop_counter = 0

    # While running
    while not rospy.is_shutdown():

      # Create the message we are going to send
      msg = RateThrust()
      msg.header.stamp = rospy.get_rostime()

      # If the drone has not been armed
      if self.armed == False:

        # Arm the drone
        msg.thrust = Vector3(0, 0, 10)
        msg.angular_rates = Vector3(0, 0, 0)   
        self.armed = True 

      # Behave normally
      else:
        # Use a PID to calculate the rates you want to publish to maintain an angle
        roll_output = self.rollPID.get_output(self.roll_setpoint, self.roll_reading)
        pitch_output = self.pitchPID.get_output(self.pitch_setpoint, self.pitch_reading)
        yaw_output = self.yawPID.get_output(self.yaw_setpoint, self.yaw_reading)

        # Create the message
        msg.thrust = Vector3(0, 0, self.thrust_setpoint)
        msg.angular_rates = Vector3(roll_output, pitch_output, yaw_output)

      # Publish the message
      self.rate_pub.publish(msg)

      # Sleep any excess time
      rate.sleep()

	# Save the new attitude readings
  def euler_angle_callback(self, msg):
    self.roll_reading = msg.x
    self.pitch_reading = msg.y
    self.yaw_reading = msg.z

	# Save the new attitude setpoints
  def attitude_set_callback(self, msg):
    # Dont allow angles greater than 0.5 for x and y
    self.roll_setpoint = max(min(msg.x, 0.5),-0.5)
    self.pitch_setpoint = max(min(msg.y, 0.5),-0.5)
    self.yaw_setpoint = msg.z

	# Save the new thrust setpoints
  def thrust_callback(self, msg):
    self.thrust_setpoint = msg.data

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('angle_controller')
  try:
    angcon = AngleController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
