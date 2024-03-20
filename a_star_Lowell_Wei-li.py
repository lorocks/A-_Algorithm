from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import cv2
import imutils
import time
import os
import math

backtrack_count = 0

def createGrid(height, width, bounding_location, padding = 0, wall = False, wall_padding = 0, travel_padding = 0, scale = 1):
  doPadding = False
  image = np.full((height, width, 3), 255, dtype=np.uint8)

  if wall and travel_padding > 0:
    image = setWallPadding(image, travel_padding, 180)
  if wall:
    image = setWallPadding(image, wall_padding, 125)

  if padding > 0 and travel_padding > 0:
    image = setObstaclesAndTruePadding(image, bounding_location, travel_padding, scale, 180)
  if padding > 0:
    image = setObstaclesAndTruePadding(image, bounding_location, padding, scale, 125)

  # image_show = cv2.flip(image, 0)
  # cv2_imshow(image_show)
  # print(image_show.shape)

  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  grid = np.full((height, width), -1)
  grid[gray == 125] = -12
  grid[gray == 180] = -15
  grid[gray == 0] = -11
  grid = grid.reshape(-1)

  return grid, image

def setWallPadding(image, padding, value):
  height, width, _ = image.shape

  points = [
      (0, 0), (padding, height),
      (0, 0), (width, padding),
      (0, height - padding), (width, height),
      (width - padding, 0), (width, height),
  ]

  for i in range(0, len(points), 2):
    cv2.rectangle(image, points[i], points[i+1], (value, value, value), -1)

  return image

def setObstaclesAndTruePadding(image, bounding_location, padding, scale, value):
  if padding > 0:
    doPadding = True
    paddings = [
        [padding, padding],
                [-padding, -padding],
     [padding, -padding],
      [-padding, padding],
                [0, padding],
                [0, -padding],
                [padding, 0],
                [-padding, 0]
                ]


  for obstacle in bounding_location:
    obstacle = np.array(obstacle, dtype=np.int32) * scale
    if len(obstacle) == 2:
      if doPadding:
        for pad in paddings:
          cv2.rectangle(image, (obstacle[0][0] - pad[0], obstacle[0][1] - pad[1]), (obstacle[1][0] + pad[0], obstacle[1][1] + pad[1]), (value, value, value), -1)
        points = [(obstacle[0][0], obstacle[0][1]), (obstacle[1][0], obstacle[1][1]), ( obstacle[0][0], obstacle[1][1]), ( obstacle[1][0],  obstacle[0][1])]
        for point in points:
          cv2.circle(image, point, padding, (value, value, value), -1)
      cv2.rectangle(image, obstacle[0], obstacle[1], (0, 0, 0), -1)
    else:
      if doPadding:
        for pad in paddings:
          length = len(obstacle)
          arrr = np.full((length, 2), pad)
          cv2.fillPoly(image, pts=[np.subtract(obstacle, arrr)], color=(value, value, value))
        for point in obstacle:
          cv2.circle(image, tuple((np.array(point))), padding, (value, value, value), -1)
      cv2.fillPoly(image, pts=[obstacle], color=(0, 0, 0))

  return image

def setGoal(grid, point, threshold):
  image = cv2.circle(grid, point, threshold, (80, 80, 80), -1) # this defo ovverides the grid :< dammit

  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  grid = np.full((height, width), -1)
  grid[gray == 125] = -12
  grid[gray == 180] = -15
  grid[gray == 0] = -11
  grid = grid.reshape(-1)

  return image

def heuristic(node, goal):
    return np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

def goal_reached(node, goal, threshold):
    if len(node) < 2 or len(goal) < 2:
        raise ValueError("node and goal must be sequences with at least two elements each")
    return heuristic(node, goal) <= threshold


def animate_search(visited, path, map_height, map_width, obstacles):
  fig, ax = plt.subplots(figsize=(12, 5)) #set animate to 12:5 match map shape
  ax.set_xlim(0, map_width) #set animate x axis
  ax.set_ylim(0, map_height) #set animate y axis

  #show obstacles
  for polygon in obstacles:
      poly = plt.Polygon(polygon, facecolor="gray", edgecolor='black')
      ax.add_patch(poly)

  points = ax.scatter([], [], s=1, color='blue')
  line, = ax.plot([], [], 'r-', lw=2)  # Path line

  def init():
      points.set_offsets(np.empty((0, 2)))
      line.set_data([], [])
      return points, line,

  def update(frame):
      skip = 3000 #set frames skip
      frame *= skip
      visited_points = np.array(visited[:frame+1]) #get visited
      points.set_offsets(visited_points)
      points.set_offsets(np.array([[6.8,6], [1194, 162]]))
      if skip > len(visited):
          global backtrack_count
          backtrack_count += 1
          skip = 20 #set flames skip
          count = backtrack_count * skip
          x, y = zip(*path[:count+1]) #get path
          line.set_data(x, y)
      return points, line,

  ani = FuncAnimation(fig, update, frames=len(visited), init_func=init, blit=True, interval=1,)
  plt.show()


start = time.time()

unscaled_robot_radius = 5
unscaled_clearance = 5
unscaled_height = 500
unscaled_width = 1200
unscaled_effective_padding = unscaled_robot_radius + unscaled_clearance

scale = 1 # test scale to see speed of iteration, (9 is like 0.0966 and 0.088) change 10

# padding = travel_dist/4 *(3 - (3)**0.5) + robot_radius * (2)**0.5

# Also, 100 > 2 (padding + travel_dist/2 * (3)**0.5)

height = unscaled_height * scale # y size
width = unscaled_width * scale # x size
effective_padding = unscaled_effective_padding * scale


travel_dist = 40 # Change to 33

# padding = int(((travel_dist * (3 - (3)**0.5)/4) + (unscaled_robot_radius * (2)**0.5)) * scale)
padding = int(((travel_dist * (3 - (3)**0.5)/4) + (unscaled_effective_padding)) * scale)

angles = [ 60, 30, 0, -30, -60 ]

timestep = 0

recording = False

obstacle_file_path = ""

obstacle_bounding_boxes = [
    [[175, 100], [100, 500] ], [[275, 400], [350, 0]],
    [[650 - (75*(3**0.5)), 325], [650 - (75*(3**0.5)), 175], [650, 100], [650 + (75*(3**0.5)), 175], [650 + (75*(3**0.5)), 325], [650, 400]],
    [[900, 450], [1100, 450], [1100, 50], [900, 50], [900, 125], [1020, 125], [1020, 375], [900, 375]],
                            ]

obstacles = [
    [(100, 100), (100, 500), (175, 500), (175, 100)],

    [(275, 0), (275, 400), (350, 400), (350, 0)],

    [(650-150*np.cos(np.pi/6), 400-150-150*0.5),
     (650-150*np.cos(np.pi/6), 400-150*0.5),
     (650, 400),
     (650+150*np.cos(np.pi/6), 400-150*0.5),
     (650+150*np.cos(np.pi/6), 400-150-150*0.5),
     (650, 100)],

    [(900, 450), (1100, 450), (1100, 50), (900, 50), (900, 125), (1020, 125), (1020, 375), (900, 375)]
]

open = PriorityQueue()
visited = []


if recording:
  size = (width, height)
  fps = 90
  record = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc(*'MJPG'), fps, size)


if os.path.exists(obstacle_file_path) and os.path.isfile(obstacle_file_path):
  pass
else:
  # Enter array manually maybe through prompt
  grid, useless = createGrid(height, width, obstacle_bounding_boxes, effective_padding, True, effective_padding, padding, scale)
  backtrack_grid = np.full((height*width), -1)

end = time.time()
print(end - start)


valid = False
while not valid:
  starting_x = int(input("\nEnter starting x position:")) * scale
  starting_y = int(input("\nEnter starting y position:")) * scale
  starting_theta = int(input("\nEnter starting theta position:"))

  current_pos = starting_x + (width * starting_y)
  try:
    if grid[current_pos] == -1:
      if not starting_theta % 30 == 0:
        continue
      grid[current_pos] = 0
      backtrack_grid[current_pos] = -1
      valid = True
    else:
      print("\nStarting position invalid, obstacle exists, Enter again\n")
  except:
    print("\nStarting position invalid, obstacle exists, Enter again\n")


valid = False
while not valid:
  goal_x = int(input("\nEnter goal x position:")) * scale
  goal_y = int(input("\nEnter goal y position:")) * scale
  goal_index = goal_x + (width * goal_y)

  try:
    if grid[goal_index] == -1:
      valid = True
    else:
      print("\nGoal position invalid, obstacle exists, Enter again\n")
  except:
    print("\nGoal position invalid, obstacle exists, Enter again\n")

# grid = setGoal(useless, (goal_x, goal_y), travel_dist/2)

# Add first node to open list
# open.put((((goal_x - starting_x)**2 + (goal_y - starting_y)**2)**0.5, current_pos, starting_theta))
# Add first node to open list
open.put(( heuristic((starting_x, starting_y), (goal_x, goal_y)), current_pos, starting_theta))

start = time.time()
while not open.empty():
  current_cost, current_pos, current_theta = open.get()

  if not grid[current_pos] == -13:
    timestep += 1
    grid[current_pos] = -13
    # print(current_pos)

    x_pos = int(current_pos % width)
    y_pos = int((current_pos - (current_pos % width))/width)

    visited.append((x_pos / scale, y_pos / scale, current_theta))

    # changes MAYBE #
    # if x_pos >= goal_x - (travel_dist/2 * scale) and x_pos <= goal_x + (travel_dist/2 * scale) and y_pos >= goal_y - (travel_dist/2 * scale) and y_pos <= goal_y + (travel_dist/2 * scale):
    if goal_reached((x_pos, y_pos), (goal_x, goal_y), travel_dist/2): # change travel/2 to a variable and keep travel/2 done before already, or goal reach into index chack yeeeehhh
      print(heuristic((x_pos, y_pos), (goal_x, goal_y)))
      last_explored = current_pos
      print("Goal path found")

      break

    for angle in angles:
      # changes MAYBE #
      new_x = x_pos + int( travel_dist * np.cos(np.deg2rad(current_theta + angle)) * scale)
      new_y = y_pos + int(travel_dist * np.sin(np.deg2rad(current_theta + angle)) * scale)
      new_pos = new_x + (width * new_y)
      # print(new_pos)
      if new_pos < 0 or new_pos > height * width:
        continue

      if grid[new_pos] < -10:
        continue
      # changes MAYBE #
      # cost = current_cost + (travel_dist * scale) - ((goal_x - x_pos)**2 + (goal_y - y_pos)**2)**0.5 + ((goal_x - new_x)**2 + (goal_y - new_y)**2)**0.5
      cost = current_cost + (travel_dist * scale) - heuristic((x_pos, y_pos), (goal_x, goal_y)) + heuristic((new_x, new_y), (goal_x, goal_y)) # maybe use wei li logic for cost? reduces mathematical operations which is something, look into it

      if grid[new_pos] == -1 or grid[new_pos] > cost:
        grid[new_pos] = cost
        backtrack_grid[new_pos] = current_pos
        open.put((cost, new_pos, current_theta + angle))


print(time.time() - start)

print("Out of while loop")


path = []
index = last_explored
last = 0
while backtrack_grid[index] > 0:
  x_pos = int(index % width)
  y_pos = int((index - (index % width))/width)
  path.append((x_pos / scale, y_pos / scale))
  # print((math.degrees(math.atan2(y_pos - int((last - (last % width))/width), x_pos - int(last % width)))))
  # last = index
  index = backtrack_grid[index]

print(len(visited))
animate_search(list(visited), path, height, width, obstacles)