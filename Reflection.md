# Path Planning Project Reflection

## Introduction

### Goals

1. Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
2. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible.
3. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
4. The car should be able to make one complete loop around the 6946m highway.
5. It should take a little over 5 minutes to complete 1 loop.
6. Car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## How it works?

  - Car drive in lane to reach the goal and maximum speed it can be go upto 49.5 mph.
  - If there is other car in lane and blocking the movement of car, then car will see if there is available lane to change. If yes, then car will keep the speed and change the lane.
  - If there is no other lane available for change, car will slow down its speed in current lane to avoid collision.

## Implementation

File : vehicle.cpp
     This file implements functions to get path for the car. It proposes multiple paths, evaluates all proposed paths and then selects the path safe to drive.

Four classes for keeping driving data
  - TrajectoryData - Trajectory specific data, referred to evaluate a trajectory
  - RouteMap - Waypoint information
  - Obstacle - Information of other vehicles on road
  - Vehicle - Information of ego car

get_path function
  - Generates multiple possible paths (function: generate_trajectory)
     - Keep lane, accelerate
     - Keep lane, same speed
     - Keep lane, deaccelerate
     - Change lane, accelerate
     - Change lane, deaccelerate
     - Change lane, same speed
     - Different spline points
  - Evaluates all proposed paths for (function: evaluate_trajectory)
     - Collision
     - Max velocity
  - If no path found then keep same lane and deaccelerate

Generating trajectory
  - Get proposed lane for the trajectory
  - If there are more than 2 waypoints in previous path then append those waypoints for current path
  - Used spline to get smoother points for the proposed trajectory

Collision detection
  - Using velocity for proposed path and other object velocity, check for all waypoints that ego vehicle and other car is not in close distance

## Result

### Observation
  - Car able to drive safely, maximum distance travelled > 9 mile without any incidence
  - Takes ~5-6 mins for one loop
  - Able to change lane smoothly
  - Able to slow down or change lane when collision detected

### Improvements
  - Use cost functions
  - More intelligent lane switching
  - Multi-lane switching

