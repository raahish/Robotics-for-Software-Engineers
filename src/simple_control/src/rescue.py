#!/usr/bin/env python
import rospy
import time
import copy
import math
from enum import Enum
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid

# A class to keep track of the drone's state
class DroneState(Enum):
  STARTED = 1
  ASCEND = 2
  TOWERSEARCH = 3
  DESCEND = 4
  SEARCH = 5
  RESCUE = 6
  RESCUED = 7

class Rescue():

    # Node initialization
    def __init__(self):

        ## CREATE PUBLISHERS
        self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
        self.goal_pub = rospy.Publisher('/uav/input/goal', Vector3, queue_size=1)
        self.cancel_pub = rospy.Publisher('/cancel/trajectory', Empty, queue_size=1)

        ## CREATE SUBSCRIBERS
        self.gps_sub = rospy.Subscriber("/uav/sensors/gps", PoseStamped, self.get_gps)
        self.hikerPos_sub = rospy.Subscriber("/tower/hiker/position", Vector3, self.get_hikerPos)
        self.hikerExactPos_sub = rospy.Subscriber("/hiker/exact/position", Vector3, self.get_hikerExactPos)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)

        ## VARIABLE DEFINITIONS
        # Create the drone's state as started
        self.state = DroneState.STARTED
        # Create the direct goal message we are going to be sending
        self.goalDirect = Vector3()
        # Create the planned goal message we are going to be sending
        self.goal = Vector3()
        # A variable that saves the drones current position
        self.drone_position = PoseStamped()
        # A variable that saves the hiker's position recieved from the tower
        self.hikerPos = PoseStamped()
        # A variable that saves the hiker's transformed position (from tower frame to world frame) recieved from the tower
        self.hikerPosTransformed = PoseStamped()
        # A variable that saves the hiker's exact position (sent by the hiker when within 5 m)
        self.hikerExactPos = PoseStamped()
        # A variable that saves all points to visit during the ground search strategy
        self.searchPoints = []
        # A variable that saves the data of a map as a 2D array
        self.map = []
        # A variable that saves width of the map
        self.mapwidth = 0

        ## BOOLEAN DEFINITIONS (for internal workings of states)
        # A bool that is True if the hiker's position is recieved from the tower
        self.hikerPosRecieved = False
        # A bool that is True if the hiker's exact position is recieved from the hiker
        self.hikerExactPosRecieved = False
        # A bool that is True if the search strategy is already planned
        self.searchPlanned = False
        # Keeps track of whether the planned goal was changed or not
        self.goal_changed = False
        # Keeps track of whether the direct goal was changed or not
        self.goal_direct_changed = False

        # Call the mainloop of our class
        self.mainloop()

    # Callback function for the /uav/sensors/gps subscriber. Recieves and stores the drone's position.
    def get_gps(self, msg):
        self.drone_position = msg

    # Callback function for the /tower/hiker/position subscriber. Recieves hiker's position from the tower and stores the
    # version recieved as well as a transformed version.
    def get_hikerPos(self, msg):
        self.hikerPos.header.stamp = rospy.Time.now()
        self.hikerPos.pose.position.x = msg.x
        self.hikerPos.pose.position.y = msg.y
        self.hikerPos.pose.position.z = msg.z
        self.hikerPosRecieved = True
        self.hikerPosTransformed.header.stamp = rospy.Time.now()
        self.hikerPosTransformed.pose.position.x = self.transform_pos(msg.x, msg.y)[0]
        self.hikerPosTransformed.pose.position.y = self.transform_pos(msg.x, msg.y)[1]
        self.hikerPosTransformed.pose.position.z = msg.z

    # Callback function for the /hiker/exact/position subscriber. Recieves hiker's position from the hiker and stores it.
    def get_hikerExactPos(self, msg):
        self.hikerExactPos.header.stamp = rospy.Time.now()
        self.hikerExactPos.pose.position.x = msg.x
        self.hikerExactPos.pose.position.y = msg.y
        self.hikerExactPos.pose.position.z = msg.z
        self.hikerExactPosRecieved = True

    # Callback function for the /map subscriber. Recieves the map, transforms the data into a 2D array and stores it.
    def get_map(self, msg):
        # Save map width
        self.mapwidth = msg.info.width
        row = []
        l = []
        # Go through each item of the the map data list and convert it into a 2D array.
        for i in range(self.mapwidth):
            for j in range(self.mapwidth):
                row.append(msg.data[self.mapwidth*i + j])
            l.append(row)
            row = []
        self.map = l

    # A function that transforms coordinates in the tower frame to coordinates in the world frame.
    def transform_pos(self, x, y):
        goalxrealmid = x + 128.575095105
        goalyrealmid = y - 77.2557112365
        goalxreal = -(goalyrealmid*math.sin(5.75959)) + goalxrealmid*math.cos(5.75959)
        goalyreal = goalyrealmid*math.cos(5.75959) + goalxrealmid*math.sin(5.75959)
        return [goalxreal, goalyreal]

    # A function that transforms coordinates to indexes on the map.
    def coordToIndex(self,x,y):
        row = int(x+self.mapwidth/2.0)
        col = int(y+self.mapwidth/2.0)
        return [row, col]

    # A function that transforms indexes on the map to coordinates in the world.
    def indextoCoord(self,row,col):
        x = row - self.mapwidth/2.0
        y = col - self.mapwidth/2.0
        return [x,y]

    # A function that returns true if there is enough space around the hiker to rescue them and false otherwise.
    def enoughSpaceAround(self, hiker):
        # Get the index on the map that contains the coordinate 3 m to the left and 3 m to the top of the hiker's position
        startRow = self.coordToIndex(hiker.pose.position.x-3, hiker.pose.position.y-3)[0]
        startCol = self.coordToIndex(hiker.pose.position.x-3, hiker.pose.position.y-3)[1]
        # Get the index on the map that contains the coordinate 3 m to the right and 3 m to the bottom of the hiker's position
        endRow = startRow + 6
        endCol = startCol + 6
        r = startRow
        c = startCol
        # Iterate through each cell on the map within 3 m from the hiker and check for obstacles
        while r <= endRow:
            while c <= endCol:
                # If the index is within map bounds
                if r < self.mapwidth and c < self.mapwidth and r >= 0 and c >= 0:
                    # If there is an obstacle at that index on the map
                    if self.map[r][c] == 100:
                        rospy.loginfo(str(rospy.get_name()) + ": OBSTACLE at [" + str(r) + ", " + str(c)+ "]")
                        return False
                c+=1
            c = startCol
            r+=1
        return True

    # A function plans a ground search after the drone has descended and not recieved the exact position of the hiker.
    def planSearch(self):
        # Create a virtual circle of radius 4 (changed from the presentation, which mentioned 5)
        radius = 4
        resolution = 0.349066 # alpha = 20 degrees
        angle = 0
        points = []
        # Find points on the circle's circumference at every angle+resolution 
        while angle <= 6.28319: # 360 degrees
            # calculate x and y coordinates at a particular angle
            x = -(radius*math.cos(angle)) + self.drone_position.pose.position.x
            y = radius*math.sin(angle) + self.drone_position.pose.position.y
            # convert coordinates to index on the map
            index = self.coordToIndex(x,y)
            # if the index is not already in the list and the index is within map bounds,
            if index not in points and index[0] >= 0 and index[0] < self.mapwidth and index[1] >= 0 and index[1] < self.mapwidth:
                # If there is no obstacle at that index on the map
                if self.map[index[0]][index[1]] == 0:
                    # Add the index to the list of indexes to visit
                    points.append(index)
            # Increment angle by resolution
            angle+=resolution

        coords = []
        # Convert all indexes into coordinates in the world
        for index in points:
            coords.append(self.indextoCoord(index[0],index[1]))

        return coords

    def obstacleAdjacent(self, index):
        if self.map[index[0]][index[1]] == 100:
            return True
        if index[0]+1 < self.mapwidth:
            if self.map[index[0]+1][index[1]] == 100:
                return True
        if index[1]+1 < self.mapwidth:
            if self.map[index[0]][index[1]+1] == 100:
                return True
        if index[1]+1 < self.mapwidth and index[0]+1 < self.mapwidth:
            if self.map[index[0]+1][index[1]+1] == 100:
                return True
        if index[0]-1 >= 0:
            if self.map[index[0]-1][index[1]] == 100:
                return True
        if index[1]-1 >= 0:
            if self.map[index[0]][index[1]-1] == 100:
                return True
        if index[0]-1 >= 0 and index[1]-1 >= 0:
            if self.map[index[0]-1][index[1]-1] == 100:
                return True
        if index[0]-1 >= 0 and index[1]+1 < self.mapwidth:
            if self.map[index[0]-1][index[1]+1] == 100:
                return True
        if index[0]+1 < self.mapwidth and index[1]-1 >= 0:
            if self.map[index[0]+1][index[1]-1] == 100:
                return True
        return False
    
    # TODO: Check if dest is adjacent to obstacle before publishing goal
    
    # TODO: Check this
    # A function that checks if there is an obstacle between two points on the map
    def obstacleBetween(self, source, dest):
        print("Going from " + str(source) + "to " + str(dest))
        x1 = source[0]
        y1 = source[1]
        x2 = dest[0]
        y2 = dest[1]
        # Calculate the angle between the two points
        alpha = abs(math.atan((x2-x1)/(y2-y1)))
        # Calculate the distance between the two points
        dist = math.hypot((x2-x1),(y2-y1))
        # resolution variable to find 50 coordinates along the path
        res = dist/50.0
        points = []
        step = 0
        # Get 50 coordinates along the path between source and dest
        while step <= dist:
            # This if-elif-else block checks the quadrant and direction and adds the step variable accordingly
            if x2-x1 < 0 and y2-y1 > 0:
                # print("top right")
                x = x1 - step*math.sin(alpha)
                y = y1 + step*math.cos(alpha)
            elif x2-x1 < 0 and y2-y1 < 0:
                # print("top left")
                x = x1 - step*math.sin(alpha)
                y = y1 - step*math.cos(alpha)
            elif x2-x1 > 0 and y2-y1 < 0:
                # print("bottom left")
                x = x1 + step*math.sin(alpha)
                y = y1 - step*math.cos(alpha)
            elif x2-x1 > 0 and y2-y1 > 0:
                # print("bottom right")
                x = x1 + step*math.sin(alpha)
                y = y1 + step*math.cos(alpha)
            elif x2-x1 == 0:
                # print("top or bottom")
                x = x1
                y = y1 - step if y2-y1 < 0 else y1 + step
            elif y2-y1 == 0:
                # print("left or right")
                x = x1 - step if x2-x1 < 0 else x1 + step
                y = y1
            else:
                print("wut?")
            # print(x,y)
            # print(self.coordToIndex(x,y))
            # If the index that the coordinate is in is not in the list and the index is on the map
            if self.coordToIndex(x,y) not in points and self.coordToIndex(x,y)[0] >= 0 and self.coordToIndex(x,y)[0] < self.mapwidth and self.coordToIndex(x,y)[1] >= 0 and self.coordToIndex(x,y)[1] < self.mapwidth:
                # Add index to the list of points to check for an obstacle
                print(self.coordToIndex(x,y))
                points.append(self.coordToIndex(x,y))
            step+=res
        
        # Check every point on the list to see if there is an obstacle in between
        for point in points:
            rospy.loginfo(str(rospy.get_name()) + ": Checking map index [" + str(point[0]) + ", " + str(point[1]) + "]")
            if self.map[point[0]][point[1]] == 100:
                rospy.loginfo(str(rospy.get_name()) + ": Obstacle in between")
                return True
        
        return False

    # This function is called when we are in the STARTED state
    def processStarted(self):
        rospy.loginfo(str(rospy.get_name()) + ": Current State: STARTED")
        # Wait for the simulator to load
        time.sleep(6)
        print(self.obstacleBetween([8,-2],[2,-3]))
        # if self.hikerExactPosRecieved:
        #     self.state = DroneState.SEARCH
        # Change state to ASCEND
        self.state = DroneState.ASCEND

    # This function is called when we are in the ASCEND state
    def processAscend(self):
        rospy.loginfo(str(rospy.get_name()) + ": Current State: ASCEND")
        # If there is a new planned goal path, wait for it to complete
        if self.goal_changed:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal")
            while abs(self.drone_position.pose.position.x - self.goal.x) >= 0.2 and abs(self.drone_position.pose.position.y - self.goal.y) >= 0.2:
                time.sleep(2)
            self.goal_changed = False
        # If there is a new direct goal path, wait for it to complete
        elif self.goal_direct_changed: 
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal direct")
            while abs(self.drone_position.pose.position.z - self.goalDirect.z) >= 0.2:
                time.sleep(2)
            self.goal_direct_changed = False
            self.goal_changed = False
            # After the drone reaches the goal altitude, change state to TOWERSEARCH
            self.state = DroneState.TOWERSEARCH
        # If there is no goal left to complete
        else:
            # If there is no goal left to complete, ascend the drone to 20.5 m
            self.goalDirect.x = self.drone_position.pose.position.x
            self.goalDirect.y = self.drone_position.pose.position.y
            self.goalDirect.z = 20.5
            self.goal_direct_changed = True

    # This function is called when we are in the TOWERSEARCH state
    def processTowerSearch(self):
        rospy.loginfo(str(rospy.get_name()) + ": Current State: TOWERSEARCH")

        if self.hikerPosRecieved == False:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting to recieve hiker pos")
            time.sleep(1)
        
        elif self.goal_direct_changed:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal direct")
            while abs(self.drone_position.pose.position.z - self.goalDirect.z) >= 0.2 and abs(self.drone_position.pose.position.x - self.goalDirect.x) >= 0.2 and abs(self.drone_position.pose.position.y - self.goalDirect.y) >= 0.2:
                time.sleep(2)
            self.goal_direct_changed = False
        
        elif self.goal_changed:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal")
            while abs(self.drone_position.pose.position.x - self.goal.x) >= 0.1 and abs(self.drone_position.pose.position.y - self.goal.y) >= 0.1:
                time.sleep(2)
            self.goal_changed = False
            rospy.loginfo(str(rospy.get_name()) + ": Hiker is right below")
            self.state = DroneState.DESCEND

        else:
            rospy.loginfo(str(rospy.get_name()) + ": Giving Goal")
            destx = self.transform_pos(self.hikerPos.pose.position.x, self.hikerPos.pose.position.y)[0]
            desty = self.transform_pos(self.hikerPos.pose.position.x, self.hikerPos.pose.position.y)[1]
            self.goal.x = destx
            self.goal.y = desty
            self.goal.z = 20.5
            self.goal_changed = True
        
    def processDescend(self):
        rospy.loginfo(str(rospy.get_name()) + ": Current State: DESCEND")
        self.cancel_pub.publish(Empty())

        if self.goal_direct_changed: 
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal direct")
            while abs(self.drone_position.pose.position.z - self.goalDirect.z) >= 0.2:
                time.sleep(2)
            self.goal_direct_changed = False
            self.goal_changed = False
            rospy.loginfo(str(rospy.get_name()) + ": Preparing to search..")
            self.state = DroneState.SEARCH

        else:
            self.goalDirect.x = self.drone_position.pose.position.x
            self.goalDirect.y = self.drone_position.pose.position.y
            self.goalDirect.z = 3
            self.goal_direct_changed = True

    def processSearch(self):
        rospy.loginfo(str(rospy.get_name()) + ": Current State: SEARCH")
        if self.hikerExactPosRecieved:
            rospy.loginfo(str(rospy.get_name()) + ": Hiker located")
            self.cancel_pub.publish(Empty())
            if self.enoughSpaceAround(self.hikerExactPos):
                rospy.loginfo(str(rospy.get_name()) + ": enough space to rescue")
                self.state = DroneState.RESCUE
                self.goal_changed = False
                self.goal_direct_changed = False
            else:
                rospy.loginfo(str(rospy.get_name()) + ": NOT enough space to rescue")
                self.state = DroneState.ASCEND
                self.goal_changed = False
                self.goal_direct_changed = False
        
        elif self.goal_direct_changed:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal direct")
            if self.hikerExactPosRecieved:
                    rospy.loginfo(str(rospy.get_name()) + ": Hiker located")
                    self.cancel_pub.publish(Empty())
            while abs(self.drone_position.pose.position.z - self.goalDirect.z) >= 0.2 and abs(self.drone_position.pose.position.x - self.goalDirect.x) >= 0.2 and abs(self.drone_position.pose.position.y - self.goalDirect.y) >= 0.2:
                time.sleep(1)
            time.sleep(2)
            self.goal_direct_changed = False
        
        elif self.goal_changed:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal")
            if self.hikerExactPosRecieved:
                    rospy.loginfo(str(rospy.get_name()) + ": Hiker located")
                    self.cancel_pub.publish(Empty())
            while abs(self.drone_position.pose.position.x - self.goal.x >= 0.2) and abs(self.drone_position.pose.position.y - self.goal.y >= 0.2):
                time.sleep(1)
            self.goal_changed = False
        # TODO: Test this part out
        else:
            rospy.loginfo(str(rospy.get_name()) + ": Hiker exact pos NOT recieved")
            # plan a circle trajectory with radius = 4 from where the drone currently is, avoiding obstacles from the occupancy grid
            if not self.searchPlanned:
                rospy.loginfo(str(rospy.get_name()) + ": Planning search trajectory")
                self.searchPoints = self.planSearch()
                sorted(self.searchPoints , key=lambda k: [k[1], k[0]])
                self.searchPlanned = True
            # if no seach points remain, change state to towersearch
            if len(self.searchPoints) == 0:
                rospy.loginfo(str(rospy.get_name()) + ": Hiker not found")
                self.state = DroneState.ASCEND
            # else, go to search 
            else:
                if self.goal_changed:
                    rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal")
                    if self.hikerExactPosRecieved:
                            rospy.loginfo(str(rospy.get_name()) + ": Hiker located")
                            self.cancel_pub.publish(Empty())
                    while abs(self.drone_position.pose.position.x - self.goal.x >= 0.2) and abs(self.drone_position.pose.position.y - self.goal.y >= 0.2):
                        time.sleep(1)
                    self.goal_changed = False
                elif self.goal_direct_changed:
                    rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal direct")
                    if self.hikerExactPosRecieved:
                            rospy.loginfo(str(rospy.get_name()) + ": Hiker located")
                            self.cancel_pub.publish(Empty())
                    while abs(self.drone_position.pose.position.z - self.goalDirect.z) >= 0.2 and abs(self.drone_position.pose.position.x - self.goalDirect.x) >= 0.2 and abs(self.drone_position.pose.position.y - self.goalDirect.y) >= 0.2:
                        time.sleep(1)
                    time.sleep(2)
                    self.goal_direct_changed = False
                
                # if no obstacle in between, take direct route
                elif not self.obstacleBetween([self.drone_position.pose.position.x,self.drone_position.pose.position.y], self.searchPoints[0]):
                    rospy.loginfo(str(rospy.get_name()) + ": Going to search point: " + str(self.searchPoints[0][0]) + ", " + str(self.searchPoints[0][1]))
                    rospy.loginfo(str(rospy.get_name()) + ": Taking direct route")
                    self.goalDirect.x = self.searchPoints[0][0]
                    self.goalDirect.y = self.searchPoints[0][1]
                    self.goalDirect.z = 3
                    self.goal_direct_changed = True
                    self.searchPoints.pop(0)
                else:
                    rospy.loginfo(str(rospy.get_name()) + ": Going to search point: " + str(self.searchPoints[0][0]) + ", " + str(self.searchPoints[0][1]))
                    rospy.loginfo(str(rospy.get_name()) + ": Taking indirect route")
                    self.goal.x = self.searchPoints[0][0]
                    self.goal.y = self.searchPoints[0][1]
                    self.goal.z = 3
                    self.goal_changed = True
                    self.searchPoints.pop(0)
    
    def processRescue(self):
        rospy.loginfo(str(rospy.get_name()) + ": Current State: RESCUE")  
        
        if self.goal_changed:
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal")
            while abs(self.drone_position.pose.position.x - self.goal.x) >= 0.2 and abs(self.drone_position.pose.position.y - self.goal.y) >= 0.2:
                time.sleep(2)
            self.goal_changed = False
            self.state = DroneState.RESCUED
        
        if self.goal_direct_changed: 
            rospy.loginfo(str(rospy.get_name()) + ": Waiting for drone to reach goal direct")
            while abs(self.drone_position.pose.position.z - self.goalDirect.z) >= 0.2:
                time.sleep(2)
            self.goal_direct_changed = False
            self.goal_changed = False
       
        else:
            self.cancel_pub.publish(Empty())
            if abs(self.drone_position.pose.position.z - 3) <= 0.3:
                rospy.loginfo(str(rospy.get_name()) + ": Descending to hiker")
                self.goalDirect.x = self.drone_position.pose.position.x
                self.goalDirect.y = self.drone_position.pose.position.y
                self.goalDirect.z = 0
                self.goal_direct_changed = True
            else:
                rospy.loginfo(str(rospy.get_name()) + ": Going to hiker")
                self.goal.x = self.hikerExactPos.pose.position.x
                self.goal.y = self.hikerExactPos.pose.position.y
                self.goal.z = self.hikerExactPos.pose.position.z
                self.goal_changed = True

    def rescued(self):
        rospy.loginfo(str(rospy.get_name()) + ": Successfully rescued the hiker!")
        quit()

    # The main loop of the function
    def mainloop(self):
        # Set the rate of this loop
        rate = rospy.Rate(5)

        # While ROS is still running
        while not rospy.is_shutdown():
        # Publish the position
            if self.goal_direct_changed:
                rospy.loginfo(str(rospy.get_name()) + "Direct Goal Sent")
                self.position_pub.publish(self.goalDirect)
                
            if self.goal_changed:
                if self.obstacleAdjacent(self.coordToIndex(self.goal.x,self.goal.y)) or self.coordToIndex(self.goal.x,self.goal.y) == self.coordToIndex(self.drone_position.pose.position.x,self.drone_position.pose.position.y):
                    rospy.loginfo(str(rospy.get_name()) + "Goal too close to obstacle or same as source, cancelled.")
                    self.goal_changed = False
                else:
                    rospy.loginfo(str(rospy.get_name()) + "Calculated Goal Sent")
                    self.goal_pub.publish(self.goal)

            # Check if the drone is in a moving state
            if self.state == DroneState.STARTED:
                self.processStarted()
            # If we are hovering then accept keyboard commands
            elif self.state == DroneState.ASCEND:
                self.processAscend()
            # If we are in a verifying state
            elif self.state == DroneState.TOWERSEARCH:
                self.processTowerSearch()

            elif self.state == DroneState.DESCEND:
                self.processDescend()

            elif self.state == DroneState.SEARCH:
                self.processSearch()
            
            elif self.state == DroneState.RESCUE:
                self.processRescue()
            
            elif self.state == DroneState.RESCUED:
                self.rescued()

            # Sleep for the remainder of the loop
            rate.sleep()


if __name__ == '__main__':
  rospy.init_node('rescue_node')
  try:
    ktp = Rescue()
  except rospy.ROSInterruptException:
    pass