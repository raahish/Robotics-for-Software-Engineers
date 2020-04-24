#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from altitude.msg import StampedPressureAltitude

import numpy as np
from Queue import LifoQueue
from enum import Enum 

class MeasurementType(Enum):
  PRESSURE_ALTITUDE = 1
  GPS_ALTITUDE = 2
  VELOCITY = 3

class Measurement:
    def __init__(self, measurement_type, value, timestamp):
        self.measurement_type = measurement_type
        self.value = value
        self.timestamp = timestamp

class KalmanFilter():

    def __init__(self):
        self.last_timestamp = None
        self.process_variance = rospy.get_param("/kalman_filter_node/process_variance", 100)
        self.gps_variance = rospy.get_param("/kalman_filter_node/gps_variance", 0.1)
        self.pressure_variance = rospy.get_param("/kalman_filter_node/pressure_variance", 0.1)
        self.velocity_variance = rospy.get_param("/kalman_filter_node/velocity_variance", 0.1)
        
        self.queue = LifoQueue()

        self.sub_pa = rospy.Subscriber('/uav/sensors/pressure_altitude',
                                      StampedPressureAltitude, self.process_pressure_altitude,
                                      queue_size = 1)
       
        self.sub_gps_a = rospy.Subscriber('/uav/sensors/gps_altitude',
                                      StampedPressureAltitude, self.process_gps_altitude,
                                      queue_size = 1)
       
        self.sub_vel = rospy.Subscriber('/uav/sensors/velocity',
                                      TwistStamped, self.process_velocity,
                                      queue_size = 1)
        self.pub =  rospy.Publisher('/uav/sensors/kalman_filter_altitude',
                                   StampedPressureAltitude,
                                   queue_size=1)
        self.mainloop()


    def process_pressure_altitude(self, msg):
        self.queue.put(Measurement(MeasurementType.PRESSURE_ALTITUDE,
                                      msg.value,
                                      msg.stamp))

    def process_gps_altitude(self, msg):
        self.queue.put(Measurement(MeasurementType.GPS_ALTITUDE,
                                      msg.value,
                                      msg.stamp))
        
    def process_velocity(self, msg):
        self.queue.put(Measurement(MeasurementType.VELOCITY,
                                      msg.twist.linear.z,
                                      msg.header.stamp))

    
    # The main loop of the node
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(1)
        # state vector
        x = np.zeros((2,1), dtype=float)
        # state transition matrix
        F = np.zeros((2,2), dtype=float)
        F[0][0] = 1
        F[1][1] = 1
        # process covariance matrix
        Q = np.zeros((2,2), dtype=float)
        # initialize P
        P = np.zeros((2,2), dtype=float)
        P[0][0] = 1
        P[1][1] = 1000
        # initialize I
        I = np.zeros((2,2), dtype=float)
        I[0][0] = 1
        I[1][1] = 1 
        # While ROS is still running
        while not rospy.is_shutdown():
            while not self.queue.empty():
                measurement = self.queue.get()
                if self.last_timestamp:
                    ########### predict step
                    delta_t = (measurement.timestamp - self.last_timestamp).to_sec()
                    # TODO: initialize matrices F and Q which depend on the delta_t
                    F[0][1] = delta_t
                    Q[0][0] = (np.power(delta_t, 4)) / 4.0
                    Q[0][1] = (np.power(delta_t, 3)) / 2.0
                    Q[1][0] = (np.power(delta_t, 3)) / 2.0
                    Q[1][1] = (np.power(delta_t, 2)) / 1.0

                    Q = Q * self.process_variance
                    
                    # TODO: update the state and the process matrix 
                    x = np.matmul(F, x)
                    P = np.add(np.matmul(np.matmul(F, P), np.transpose(F)), Q)

                    ########### update step
                    gps_check = measurement.measurement_type == MeasurementType.GPS_ALTITUDE
                    prs_check = measurement.measurement_type == MeasurementType.PRESSURE_ALTITUDE
                    vel_check = measurement.measurement_type == MeasurementType.VELOCITY

                    if gps_check:
                        z = np.zeros((1,1), dtype=float)
                        z[0] = measurement.value
                        H = np.zeros((1,2), dtype=float)
                        H[0][0] = 1
                        R = np.zeros((1,1), dtype=float)
                        R[0] = self.gps_variance

                    elif prs_check:
                        z = np.zeros((1,1), dtype=float)
                        z[0] = measurement.value
                        H = np.zeros((1,2), dtype=float)
                        H[0][0] = 1
                        R = np.zeros((1,1), dtype=float)
                        R[0] = self.pressure_variance

                    elif vel_check:
                        z = np.zeros((1,1), dtype=float)
                        z[0] = measurement.value
                        H = np.zeros((1,2), dtype=float)
                        H[0][1] = 1
                        R = np.zeros((1,1), dtype=float)
                        R[0] = self.velocity_variance
                    else:
                        exit()

                    # TODO: implement the update equations 
                    y = z - np.matmul(H, x)
                    # print("y: " + str(np.shape(y)))
                    S = np.matmul(np.matmul(H, P), H.T) + R
                    # print("----------------")
                    # print("H: " + str(np.shape(H)))
                    # print("H.T: " + str(np.shape(H.T)))
                    # print("P: " + str(np.shape(P)))
                    # print("R: " + str(np.shape(R)))
                    # print("----------------")
                    # print("S: " + str(np.shape(S)))
                    K = np.matmul(np.matmul(P, H.T), (np.power(S, -1)))
                    # print("K: " + str(np.shape(K)))
                    x = x + np.matmul(K, y)
                    # print("x: " + str(np.shape(x)))
                    P = np.matmul((I - np.matmul(K, H)), P)
                    # print("P: " + str(np.shape(P)))
                else:
                    # first measurement
                    if measurement.measurement_type == MeasurementType.VELOCITY:
                        x[1][0] = measurement.value
                    else:
                        x[0][0] = measurement.value
                # update timestamp
                self.last_timestamp = measurement.timestamp
                    
                # publish message
                msg = StampedPressureAltitude()
                msg.value = x[0][0]
                msg.stamp = self.last_timestamp
                self.pub.publish(msg)

            # Sleep for the remainder of the loop
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('kalman_filter_node')
    try:
        KalmanFilter()
    except rospy.ROSInterruptException:
        pass