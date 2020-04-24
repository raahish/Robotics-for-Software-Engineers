#!/usr/bin/env python
import rospy
import tf2_ros
import time
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

class TowerToMap:

  def __init__(self):
    time.sleep(10)
    # Used by the callback for the topic /tower/goal
    self.goal = None
    # Instantiate the Buffer and TransformListener
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    # Goal publisher on topic /uav/input/goal
    self.goalpub = rospy.Publisher('/uav/input/goal', Vector3, queue_size=1)
    # Tower goal subscriber to topic /tower/goal
    self.goalsub = rospy.Subscriber("/tower/goal", Vector3, self.get_goal)

    # start main loop
    self.mainloop()

  # Callback for the tower goal subscriber 
  def get_goal(self, msg):
    self.goal = msg
    rospy.loginfo(str(rospy.get_name()) + ": Recieved Goal from tower: " + str(self.goal))


  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
      if self.goal:
        try: 
            transform = self.tfBuffer.lookup_transform('world', 'tower', rospy.Time())

            # msg = Vector3()
            # msg.x = self.goal.x
            # msg.y = self.goal.y
            # msg.z = self.goal.z

            # x = (msg.y * math.sin(0.523599)) + (msg.x * math.cos(0.523599))
            # y = (msg.y * math.cos(0.523599)) - (msg.x * math.sin(0.523599))
            
            # msg.x = round(x + (150*math.sin(0.506145)) - 0.370243036950555)
            # msg.y = round(y - (150*math.cos(0.506145)) + 0.509702033064983)

            # # msg.y = y*math.cos(0.523599) - x*math.sin(0.523599)
            # # msg.x = y*math.sin(0.523599) + x*math.cos(0.523599)
            # msg.z = 0
        # Convert the goal to a PointStamped
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.point.x = self.goal.x
            point.point.y = self.goal.y
            point.point.z = self.goal.z
        # Use the do_transform_point function to convert the point using the transform
            new_point = do_transform_point(point, transform)
        # Convert the point back into a vector message containing intergers
            msg = Vector3()
            msg.x = new_point.point.x
            msg.y = new_point.point.y
            msg.z = new_point.point.z
        # Publish the vector
            rospy.loginfo(str(rospy.get_name()) + ": Publishing Transformed Goal {}".format([msg.x, msg.y]))
            self.goalpub.publish(msg)
        # The tower will automatically send you a new goal once the drone reaches the requested position.
        # Reset the goal
            self.goal = None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          continue
        
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('tower_to_map')
  try:
    tom = TowerToMap()
  except rospy.ROSInterruptException:
    pass