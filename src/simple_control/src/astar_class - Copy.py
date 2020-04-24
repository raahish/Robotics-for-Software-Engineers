from Queue import PriorityQueue
import copy 
import math
from heapq import heappush, heappop

class AStarPlanner:

# Find a plan through the map from drone_position to goal_position
# Mapdata         : List of lists where each row in the occupancy grid is a list. map_data[0][0] will give you the first cell
# Drone Position  : List with two integers. drone_position[0] will give you the x data
# Goal Position   : List with two integers. goal_position[0] will give you the x data
def AStarPlan(map_data, drone_position, goal_position):
    g_cost = 0
    h_cost = euclidean_distance(drone_position, goal_position)
    f_cost = h_cost + g_cost

    # A list to keep track of all nodes to visit
    toVisit = [(f_cost, g_cost, drone_position, None)]
    # A dictionary to keep track of what node came before a certain node
    came_from = {}
    # A list to keep track of visited nodes
    visited = []

    # While there are nodes to visit
    while toVisit:
        # get smallest item and remove from toVisit based on the f_cost.
        element = heappop(toVisit)
        total_cost, cost, pos, previous = element

        # if this has been visited already, skip it
        if pos in visited:
            continue

        visited.append(pos)

        # set its previous node
        came_from[str(pos)] = previous

        # if the goal has been reached, we are done!
        if pos == goal_position:
          break

        # check all neighbors
        for new_pos in get_neighbors(pos, map_data):
            # If the neighboring node is diagonal to the current node, the cost is sqrt(10^2+10^2) ~ 14
            if isDiagonal(pos, new_pos):
                deltacost = 14
            # If the neighboring node is not diagonal, the cost is 10
            else:
                deltacost = 10

        if (new_pos not in visited):
            new_g_cost = cost + deltacost
            new_h_cost = distance(new_pos, goal)
            new_f_cost = new_g_cost + new_h_cost
            heappush(toVisit, (new_f_cost, new_g_cost, new_pos, pos))

    path = []
    if pos == goal_position:
        path.append(pos)
        # Iterate through the came_from dictionary to get the path
        while pos:
            pos = came_from[str(pos)]
            path.append(pos)
        # Remove the last None element
        path.pop()
        # reverse so that path is from start to goal.
        path.reverse()
    return path

  def distance(self, p1,p2):
    return math.sqrt((p1[1]-p2[1])**2 + (p1[0]-p2[0])**2)

  def isDiagonal(self, pos, new_pos):
    if pos[0] - new_pos[0] == 0 or pos[1] - new_pos[1] == 0:
      return False
    else:
      return True

  # Get the children nodes for the current node
  def get_neighbors(self, node_in, map_in):
    # A list of neighbors
    neighbors = []
    # Get the current position
    pos = node_in
    # For all adjacent values
    for x_dim in range(-1, 2):
      for y_dim in range(-1, 2):
        # If the values are not 0, 0 (which is itself)
        if not (x_dim == 0 and y_dim == 0):
          # If the values are inside the map
          if (0 <= pos[0] + x_dim < map_in.shape[0]) and(0 <= pos[1] + y_dim < map_in.shape[1]):
            # If that value is an open square on the map
            if map_in[pos[0] + x_dim][pos[1] + y_dim] == 0:
              # Create a neighbor with the new position
              n = [pos[0] + x_dim, pos[1] + y_dim]
              # Save the neighbors
              neighbors.append(n)
    return neighbors
    
  # Validate the incoming data
  def validate_data(self, map_data, drone_position, goal_position):
    # Confirm that the map has two dimensions
    assert(len(map_data.shape) == 2)
    # Confirm that the drone and goal position are two points
    assert(len(drone_position) == 2)
    assert(len(goal_position) == 2)
    # Confirm that all positions are integers
    for x in drone_position + goal_position:
      assert(type(x) == int)
    # Confirm the both the start and end goal lie inside the map
    assert(0 <= drone_position[0] < map_data.shape[0])
    assert(0 <= drone_position[1] < map_data.shape[1])
    assert(0 <= goal_position[0] < map_data.shape[0])
    assert(0 <= goal_position[1] < map_data.shape[1])
    # Confirm that the drone_position and the goal position are not the same
    assert(not (drone_position[0] == goal_position[0] and drone_position[1] == goal_position[1]))
    # Confirm that the goal and drone position are free space
    assert(map_data[drone_position[0], drone_position[1]] == 0)
    assert(map_data[goal_position[0], goal_position[1]] == 0)

  # Expand the obstacles by distance so you do not hit one
  def expand_obstacles(self, map_data, distance):
    new_map = copy.deepcopy(map_data)
    # For each element in the map
    for i in range(map_data.shape[0]):
      for j in range(map_data.shape[1]):
        # if this is an obstacle
        if map_data[i, j] != 0:
          # Expand the obstacle by 1 in all directions
          for x_dim in range(-1, 2):
            for y_dim in range(-1, 2):
              if (0 <= i + x_dim < new_map.shape[0]) and(0 <= j + y_dim < new_map.shape[1]):
                new_map[i + x_dim, j + y_dim] = map_data[i, j]
    return new_map


