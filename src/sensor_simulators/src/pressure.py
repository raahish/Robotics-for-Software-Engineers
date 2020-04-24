#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import random
from sensor_simulators.srv import calibrate, calibrateResponse

# Create a class which saves the altitude of the drone and esimates the pressure
class PressureSensor():

  # Node initialization
  def __init__(self):

    # Create the publisher and subscriber
    self.pressure_pub = rospy.Publisher('/uav/sensors/pressure', Float64, queue_size=1)
    self.position_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.getPosition, queue_size = 1)
    
    self.service = rospy.Service('calibrate_pressure', calibrate, self.CalibrateFunction)

    # Save the altitude of the drone
    self.altitude = 0

    # Used to estimate the pressure
    self.pressure = 0

    # Determines the baseline value of the perssure sensor at height 0m
    self.baseline_value = 0

    # Call the mainloop of our class
    self.mainloop()


  # Callback for the keyboard manager
  def getPosition(self, msg):
    # Save the drones alitude
    self.altitude = msg.pose.position.z


  # Saves the baseline value
  def CalibrateFunction(self, request):
    # If we want to calibrate
    if request.zero == True:
      self.baseline_value = self.pressure
    else:
      self.baseline_value = 0
    # Return the new baseline value
    return calibrateResponse(self.baseline_value)


  # The main loop of the function
  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(0.25)

    # Message for sending data
    pressure_msg = Float64()

    # While ROS is still running
    while not rospy.is_shutdown():

      # Publish the altitude
      self.pressure_pub.publish(pressure_msg)

      # Compute the pressure in milibars (according to https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf)
      h = self.altitude
      common = pow((44307.7 - h),(12145.0/47571.0))
      print("---------------------------------")
      self.pressure = (3.36124)*(pow(10,-7))*(pow(h,2))*(common) - (0.00744645)*(h)*(common) + (65.987)*(common) 

      # Add sensor noise
      noise = 0

      # Set the pressure
      pressure_msg.data = self.pressure + noise - self.baseline_value

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('pressure_simulation_node')
  try:
    ktp = PressureSensor()
  except rospy.ROSInterruptException:
    pass