# Path Planning Project Reflection

## Introduction

### Goals

1. Safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
2. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible.
3. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
4. The car should be able to make one complete loop around the 6946m highway.
5. It should take a little over 5 minutes to complete 1 loop.
6. Car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Approach

Generate multiple possible trajectories
  - Keep lane, accelerate
  - Keep lane, deaccelerate
  - Keep lane, same speed
  - Change lane, accelerate
  - Change lane, deaccelerate
  - Change lane, same speed

Also, used different spline points to get smooth trajectory for different situation.

After that eveluate these trajectories for different criterias such as collision, speed etc. Currently using checks for evaluating but cost functions can also be used.

## Result

### Observation
  - Car able to driver safely, maximul distance travelled > 9 mile without any incidence
  - Takes ~5-6 mins for one loop
  - Able to change lane smoothly
  - Able to slow down or change lane when collision detected

### Improvements
  - Use cost functions
  - More intelligent lane switching

