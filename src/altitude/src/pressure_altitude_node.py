#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from altitude.msg import StampedPressureAltitude

class PressureAltitude():

    # Node initialization
    def __init__(self):
        # create the message first to avoid race conditions with the 
        # subscriber callback
        self.altitude = StampedPressureAltitude()
        # Create the publisher 
        self.altitude_pub = rospy.Publisher('/uav/sensors/pressure_altitude', StampedPressureAltitude, queue_size=1)
        # Create the subscriber
        self.pressure_sub = rospy.Subscriber('/uav/sensors/pressure', Float64, self.set_altitude, queue_size = 1)
        self.mainloop()
        
    # Callback function to calculate the pressure altitude and publish the message
    def set_altitude(self, msg):
        self.altitude = StampedPressureAltitude()
        # TODO update self.altitude message with the pressure altitude calculated
        self.altitude.value = (1 - pow(msg.data/1013.25, 0.190284)) * 145366.45 * 0.3048
        self.altitude.stamp = rospy.get_rostime()
        #time.time()
        # from the pressure in msg, and with current time stamp
        self.altitude_pub.publish(self.altitude)

    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(1)
        # While ROS is still running
        while not rospy.is_shutdown():
            # Sleep for the remainder of the loop
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pressure_altitude_node')
    try:
        PressureAltitude()
    except rospy.ROSInterruptException:
        pass