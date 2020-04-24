#!/usr/bin/env python
import rospy
import time
import copy
import math

from keyboard.msg import Key
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64


# Create a class which we will use to take keyboard commands and convert them to a position
class KeyboardManager():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/keyboardmanager/position', Vector3, queue_size=1)
    self.keyboard_sub = rospy.Subscriber('keyboard/keydown', Key, self.get_key, queue_size=1)
    self.yaw_pub = rospy.Publisher('/uav/input/yaw', Float64, queue_size=1)

    # Create the position message we are going to be sending
    self.pos = Vector3()
    self.prev_pos = Vector3()

    # Create the yaw variables
    self.yaw = 0
    self.prev_yaw = 0

    # Start the drone a little bit off the ground
    self.pos.z = 2.5
    # Create a variable we will use to hold the key code
    self.key_code = -1
    # Call the mainloop of our class
    self.mainloop()


  # Callback for the keyboard sub
  def get_key(self, msg):
    self.key_code = msg.code


  # Converts a position to string for printing
  def goalToString(self, pos, yaw):
    pos_str = "Position(" + str(pos.x) 
    pos_str += ", " + str(pos.y)
    pos_str += ", " + str(pos.z) + ")"
    pos_str += " Yaw(" + str(round(yaw,2)) + ")"
    return pos_str


  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(2)

    # While ROS is still running
    while not rospy.is_shutdown():

      # Check if any key has been pressed
      if self.key_code == 273:
        # "up" key was pressed
        self.pos.x += 0.5

      elif self.key_code == 274:
        # "down" key was pressed
        self.pos.x -= 0.5

      elif self.key_code == 275:
        # "left" key was pressed
        self.pos.y += 0.5

      elif self.key_code == 276:
        # "right" key was pressed
        self.pos.y -= 0.5
        
      elif self.key_code == 117:
        # "u" key was pressed
        self.pos.z += 0.5

      elif self.key_code == 100:
        # "d" key was pressed
        self.pos.z -= 0.5

      elif self.key_code == 108:
        # "l" key was pressed
        self.yaw += math.pi / 4.0
        if self.yaw >= math.pi:
          self.yaw = math.pi - 0.1

      elif self.key_code == 114:
        # "r" key was pressed
        self.yaw -= math.pi / 4.0
        if self.yaw <= -1 * math.pi:
          self.yaw =  -1 * math.pi + 0.1

      elif self.key_code != 13 and self.key_code != -1:
        rospy.loginfo(str(rospy.get_name()) + ": Keys assigned are: 'arrows' -> position, 'u' -> up, 'd' -> down, 'l' -> yaw left, 'r' -> yaw right")

      # Check if the position has changed
      check_x = self.prev_pos.x != self.pos.x
      check_y = self.prev_pos.y != self.pos.y
      check_z = self.prev_pos.z != self.pos.z
      check_w = self.prev_yaw != self.yaw
      if check_x or check_y or check_z or check_w:
        self.prev_pos = copy.deepcopy(self.pos)
        self.prev_yaw = self.yaw
        rospy.loginfo(str(rospy.get_name()) + ": Keyboard: " + self.goalToString(self.pos, self.yaw))

      if self.key_code == 13:
        # Publish the position
        self.position_pub.publish(self.pos)
        self.yaw_pub.publish(self.yaw)
        rospy.loginfo(str(rospy.get_name()) + ": Sending Position")

      # Reset the code
      if self.key_code != -1:
        self.key_code = -1

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('keyboard_manager')
  try:
    ktp = KeyboardManager()
  except rospy.ROSInterruptException:
    pass