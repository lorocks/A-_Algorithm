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

## User Inputs
The program will prompt you to add the starting location as, (x, y, ϴ)
x - starting x location
y - starting y location
ϴ - starting yaw angle orientation 

Next, the goal location needs to be input, (x, y, ϴ)
x - goal x location
y - goal y location
ϴ - goal yaw angle orientation 

Next, the robot radius needs to be input

Followed by the clearance between obstacle and robot

Finally, the step size needs to be input

## Code Execution
The code can be executed in any way.

After code execution takes place, a video showing the search will pop up.

