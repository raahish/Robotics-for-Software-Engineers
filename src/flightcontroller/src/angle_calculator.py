#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock


class AngleCalculator():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 20.0
    self.dt = 1.0 / self.rate

    # Create the subscribers and publishers
    self.att_pub = rospy.Publisher('/uav/sensors/attitude', Vector3, queue_size=1)
    self.imu_sub = rospy.Subscriber("/uav/sensors/filtered_imu", Imu, self.imu_callback)

    self.quarternion_pose=(0,0,0,0)

    # Run the node
    self.Run()


  # This is the main loop of this class
  def Run(self):
     # Set the rate
    rate = rospy.Rate(self.rate)

    # While running
    while not rospy.is_shutdown():

      # Convert quaternion to euler
      euler = euler_from_quaternion(self.quarternion_pose)
      # Publish the Euler Angle
      msg = Vector3(euler[0], euler[1], euler[2])
      self.att_pub.publish(msg)

      # Sleep any excess time
      rate.sleep()


  # Call back to get the GPS data
  def imu_callback(self, gps_msg):
    # Get the quarternion message
    self.quarternion_pose = (gps_msg.orientation.x,gps_msg.orientation.y,gps_msg.orientation.z,gps_msg.orientation.w)


  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node("Angle_Calculator_Node")
  try:
    calc = AngleCalculator()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()