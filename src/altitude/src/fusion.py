#!/usr/bin/env python
import rospy
import time
from threading import Lock
from altitude.msg import StampedPressureAltitude

class AltitudeFusion():

    # Node initialization
    def __init__(self):
        self.pressure_altitude = 0
        self.gps_altitude = 0
        self.last_timestamp = None
        self.changed = False
        self.lock = Lock()        
        # Create the publisher and subscriber
        self.pub = rospy.Publisher('/uav/sensors/altitude_ma',
                                   StampedPressureAltitude,
                                   queue_size=1)
        self.pa_sub = rospy.Subscriber('/uav/sensors/pressure_altitude_ma',
                                    StampedPressureAltitude, self.process_pressure_altitude,
                                    queue_size = 1)
        self.pa_sub = rospy.Subscriber('/uav/sensors/gps_altitude_ma',
                                    StampedPressureAltitude, self.process_gps_altitude,
                                    queue_size = 1)

        self.mainloop()

    def process_pressure_altitude(self, msg):
        self.lock.acquire()
        self.changed = True
        self.pressure_altitude = msg.value
        self.lock.release()

    def process_gps_altitude(self, msg):
        self.lock.acquire()
        self.changed = True
        self.gps_altitude = msg.value
        self.lock.release()

        
    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(1)
        # While ROS is still running
        while not rospy.is_shutdown():
            avg_msg = None
            self.lock.acquire()
            if self.changed:
                avg_msg = StampedPressureAltitude()
                avg_msg.value = (self.pressure_altitude + self.gps_altitude) / 2.0
                self.changed = False
            self.lock.release()
            if avg_msg:
                self.pub.publish(avg_msg)
            # Sleep for the remainder of the loop
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('altitude_ma_node')
    try:
        AltitudeFusion()
    except rospy.ROSInterruptException:
        pass