# A*_Algorithm
The following implementation uses A* algorithm to find the optimal path to the goal in a 2D graph given a restriced action set.

## GitHub Link
https://github.com/lorocks/A-_Algorithm

## Installing Dependencies
```bash
# Install numpy
  pip install numpy
# Install opencv
  pip install opencv-python
```

## Action Set
The action set consists to 5 actions of equal cost but with different angles.
Angles - [-60, -30, 0, 30, 60]

## Code Files
There are two code files, one find the optimal path to the goal point and the other finds optimal path while keeping in mind the required goal orientation.

1. a_star_Lowell-Wei-Li_optimal_path.py - Only finds optimal path to goal point
2. a_star_Lowell_Wei-Li_all_angles.py - Finds optimal path for the given goal point and goal orientation

## User Inputs
The program will prompt you to add the robot radius

Followed by the clearance between obstacle and robot

Next, the step size needs to be input

The program will prompt you to add the starting location as, (x, y, ϴ)
x - starting x location
y - starting y location
ϴ - starting yaw angle orientation 

Next, the goal location needs to be input, (x, y, ϴ)
x - goal x location
y - goal y location
ϴ - goal yaw angle orientation 


## Code Execution
The code can be executed in any way.

After code execution takes place, a video showing the search will pop up.


## Project Video
The following video shows the grid search for starting location (50, 60, 0) and goal location (1160, 450, 210)


https://github.com/lorocks/A-_Algorithm/assets/63993526/12b63636-c9c7-477a-9847-0ddc5b24d36d

