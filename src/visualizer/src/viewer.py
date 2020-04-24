#!/usr/bin/env python
import rospy
import math
import numpy as np
import copy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Vector3

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from math import sin, cos, degrees, sqrt
from tf.transformations import euler_from_quaternion


class Viewer():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 1
    self.dt = 1.0 / self.rate

    # Create the drone and hiker position
    self.drone_current = Pose()
    self.hiker_position = Pose()
    self.hiker_position.position.x = -100
    self.hiker_position.position.y = -100
    self.hiker_position.position.z = -100

    # Look for the hikers exact position
    self.hiker_exact_position = [15, 15, 0]
    self.exact_update = False

    # Create the of the map dimensions
    self.start_x = -5
    self.start_y = -5
    self.end_x = 5
    self.end_y = 5

    # Create the trajectory
    self.traj = np.array([[0, 0]])
    self.traj_height= 25.0

    # Hit Wall
    self.collision_detected = False

    # Create the viewer
    self.fig = plt.figure()
    self.ax = self.fig.gca(projection='3d')
    self.ax.set_xlim([self.start_x, self.end_x])
    self.ax.set_ylim([self.start_y, self.end_y])
    self.ax.set_zlim([0, 30])
    self.ax.set_title("Pos: (" + str(round(0)) + ", " + str(round(0)) + ", " + str(round(0)) + ")")
    self.scat, = self.ax.plot([0], [0], [0], marker=".", linestyle="", markersize=15, color='r')
    self.scat2, = self.ax.plot([0], [0], [0], marker=".", linestyle="", markersize=15, color='black')
    self.line, = self.ax.plot(self.traj[:,0], self.traj[:,1], np.full(len(self.traj[:,1]), self.traj_height), marker=".", linestyle="--", markersize=10, color='g')
    self.ax.set_xlabel('X Axis')
    self.ax.set_ylabel('Y Axis')
    self.ax.set_zlabel('Z Axis')
    plt.ion() 
    plt.show()

    # Keep track if we need to update the axis
    self.updateaxis = False

    # Keep track of the obstacles
    self.obstacle_list = []
    
    # Create the subscribers
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
    self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
    self.hiker_sub = rospy.Subscriber("/hiker/position", Pose, self.get_hiker)
    self.hiker_exact_sub = rospy.Subscriber("/hiker/exact/position", Vector3, self.get_exact_hiker)
    self.trajectory_sub = rospy.Subscriber('/uav/trajectory', Int32MultiArray, self.get_traj)

    # Collision publisher
    self.col_pub = rospy.Publisher('/uav/collision', Empty, queue_size=1)

    # Run the communication node
    self.DrawLoop()


  # Callback: Get the drone position
  def get_gps(self, msg):
    self.drone_current.position = msg.pose.position
    self.drone_current.orientation = msg.pose.orientation

  # Callback: Get the hikers position
  def get_hiker(self, msg):
    self.hiker_position.position = msg.position

  # Callback: Get the hikers exact position
  def get_exact_hiker(self, msg):
    self.hiker_exact_position = [msg.x, msg.y, msg.z]
    self.exact_update = True

  # Callback: Get the current trajectory
  def get_traj(self, msg):
    # Get the trajectory
    self.traj = np.array(np.reshape(msg.data, (-1, 2)))
    self.traj[:, 0] = self.traj[:, 0]
    self.traj[:, 1] = self.traj[:, 1]
    # We are using the offset as the height
    self.traj_height = float(msg.layout.data_offset)

  # Callback: Get the map
  def get_map(self, msg):
    # Get the map
    self.updateaxis = True
    # Get the dimensions
    width = msg.info.width
    height = msg.info.height
    res = msg.info.resolution
    # Get the origin
    self.start_x = msg.info.origin.position.x
    self.start_y = msg.info.origin.position.y
    self.end_x = self.start_x + (width * res)
    self.end_y = self.start_y + (height * res)
    # Turn the map into the right shape
    map_data = np.reshape(msg.data, (width, height))
    # Get a list of obstacles
    for xi in range(0, width):
      for yi in range(0, height):
        if map_data[xi, yi] > 50:
          self.obstacle_list.append((xi + self.start_x, yi + self.start_y))


  # This is the main loop of this class
  def DrawLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # While running
    while not rospy.is_shutdown():

        # Display the position
        self.view_point()

        # Sleep any excess time
        rate.sleep()


	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


  def insideObstacle(self):
    inside = False
    for obs in self.obstacle_list:
      if (obs[0] < self.drone_current.position.x < obs[0] + 1) and (obs[1] < self.drone_current.position.y < obs[1] + 1):
        inside = True
        break

    return inside


  def view_point(self):
    euler = euler_from_quaternion([
      self.drone_current.orientation.x,
      self.drone_current.orientation.y,
      self.drone_current.orientation.z,
      self.drone_current.orientation.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    new_x = sin(yaw)
    new_y = cos(yaw)

    x = self.drone_current.position.x
    y = self.drone_current.position.y
    z = self.drone_current.position.z

    hiker_x = self.hiker_position.position.x
    hiker_y = self.hiker_position.position.y
    hiker_z = self.hiker_position.position.z
    
    redraw_all = False
    draw_height = 0

    # Compute distance to the drone and person
    dx = self.hiker_exact_position[0] - x
    dy = self.hiker_exact_position[1] - y
    dz = self.hiker_exact_position[2] - z
    drone_hiker_distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2))

    # Update the title
    if drone_hiker_distance < 1 and -0.5 < z < 0.5:
      
      # Get the centers of the obstacles
      center_obstacles = np.array(self.obstacle_list) + [0.5, 0.5]
      closest_obstacle = 10000

      # Compute the closest obstacle
      for cen_obs in center_obstacles:
        dx = self.hiker_exact_position[0] - cen_obs[0]
        dy = self.hiker_exact_position[1] - cen_obs[1]
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if dist < closest_obstacle:
          closest_obstacle = dist

      if closest_obstacle >= 3:
        self.ax.title.set_text("Rescue Succesful!")
      else:
        self.ax.title.set_text("Too Dangerous to Rescue - Obstacle Distance "+ str(round(closest_obstacle, 1)))
    else:
      self.ax.title.set_text("Pos: (" + str(round(x,2)) + ", " + str(round(y,2)) + ", " + str(round(z,2)) + ")")

    # Check if we need to update the xlim and ylim
    if self.updateaxis:
      self.ax.set_xlim([self.start_x, self.end_x])
      self.ax.set_ylim([self.start_y, self.end_y])
      self.ax.set_zlim([0, 30])

    if self.exact_update:
      self.exact_update = False
      hiker_x = self.hiker_exact_position[0]
      hiker_y = self.hiker_exact_position[1]
      hiker_z = self.hiker_exact_position[2]
      hiker_color = 'blue'
    else:
      hiker_color = 'black'

    # Draw the quiver and scatter and keep track of where they are in the collections
    #draw_position = len(self.ax.collections)
    #self.ax.quiver(x, y, z, new_x, new_y, 0, color='r')
    self.scat.set_xdata([x])
    self.scat.set_ydata([y])
    self.scat.set_3d_properties([z])
    self.scat2.set_xdata([hiker_x])
    self.scat2.set_ydata([hiker_y])
    self.scat2.set_3d_properties([hiker_z])
    self.scat2.set_color(hiker_color)
    self.line.set_xdata(self.traj[:,0])
    self.line.set_ydata(self.traj[:,1])
    self.line.set_3d_properties(np.full(len(self.traj[:,1]), self.traj_height))

    # Only draw obstacles if they have not been drawn before
    if len(self.ax.collections) < len(self.obstacle_list):
      draw_height = self.drone_current.position.z
      # Draw obstacles
      for obs in self.obstacle_list:
        obs_x = obs[0]
        obs_y = obs[1]
        x_obs = [obs_x, obs_x + 1, obs_x + 1, obs_x]
        y_obs = [obs_y, obs_y, obs_y + 1, obs_y + 1]
        z_obs = [draw_height, draw_height, draw_height, draw_height]
        verts = [list(zip(x_obs, y_obs, z_obs))]
        self.ax.add_collection3d(Poly3DCollection(verts))
        redraw_all = True

    # Check for collisions
    if self.insideObstacle() and not self.collision_detected:
      self.collision_detected = True

    if self.collision_detected:
      self.ax.title.set_text("Obstacle Hit")

    # Draw the plot
    if redraw_all:
      self.fig.canvas.draw()
      self.fig.canvas.flush_events()
    else:
      self.ax.draw_artist(self.ax.patch)
      self.ax.draw_artist(self.scat)
      self.ax.draw_artist(self.scat2)
      self.ax.draw_artist(self.line)
      #self.fig.canvas.update()
      self.fig.canvas.flush_events()      

    for spine in self.ax.spines.values(): self.ax.draw_artist(spine)

    # Remove the obstacles if the drone has changed height significantly
    if abs(draw_height - z) > 0.5:
      # while len(self.ax.collections) > 0:
      self.ax.collections = []



def main():
  rospy.init_node('viewer_node')
  try:
    v = Viewer()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
