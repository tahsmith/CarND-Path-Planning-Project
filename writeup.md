# Path Planning Project

## State

The FSM that decides new trajectories has 6 states:

 * START: The initial state, at rest in lane 1. N.B. lanes have been labelled 
 from zero. This state cannot re reentered once left.
 * SPEED_UP: Stay in the same lane but set the speed target higher.
 * SLOW_DOWN: Stay in the same lane but slow down.
 * CRUISE: Stay in the same lane at a speed safe for the lane.
 * CHANGE_LEFT: Change one lane left at a speed safe for the lane.
 * CHANGE_RIGHT: Change one lane right at a speed safe for the lane.
 
The speed up state allows for a graceful start up. The slow down state allows
for caution in the case that there are no other good options available.
The system typically spends most of the time in the cruise state.

## Choosing the next state

### Cost components

All components of the final cost of a state have been normalised to the 0, 1
interval. After this scales are applied. This helps enforce a hierarchy of 
costs. Physical/safety limits like car collision have been given a scale of 1e9.
Law limits have been given a scale of 1e6. Comfort limits have been given 1e3.

 * Car collision: This is a hard wall potential that stops the car bodies 
 colliding.
 * Valid lane: Ensures the cars do not change lanes off the track
 * Speed limit: Stay below the hard speed limit.
 * Speed cost: Stay close to the soft speed limit.
 * Keep right: Stay on the right lane if possible. This weight is only slightly
 greater than the cost of changing lanes, so it will only happen if nothing else
 is going on.
 * Car avoidance: Make sure there is a safe gap between cars.
 * Lane change: Don't make unnecessary lane changes.
 
## Trajectory Generation
 
### Own car
 
To generate trajectories for the car under control. The first three point of 
Last path are taken to calculate an initial velocity and acceleration so
the new curve joins smoothly to the last one. Final states are chosen by
transforming desired final state s,d coords to x,y coords. The road normal is
used to find a final velocity. (See PathPlanner::GenerateTrajectoryFromCurrent.)
A jerk minimal polynomial is used at the interpolation curve between the start
and end points.

The trajectory is then filtered to ensure it does not exceed the constraints at
any point.

Road points are also interpolated with polynomial, instead of the linear routine
provided. This allowed from smoother curves in corners. (See MapData class.)

### Other cars

Other cars on the road are assumed to occupy the same lane and travel at the 
same speed. Their trajectories are calculated use the same JMT. 
(See PathPlanner::UpdateCarPaths).