# Path Planning Project

## State

The FSM that decides new trajectories has 6 states:

 * START: The initial state, at rest in lane 1. N.B. lanes have been labelled 
 from zero. This state cannot re reentered once left.
 * CAUTION: Travel at a "caution" speed, 75% of the speed limit.
 * CRUISE: Stay in the same lane at a speed safe for the lane.
 * CHANGE_LEFT: Change one lane left at a speed safe for the lane.
 * CHANGE_RIGHT: Change one lane right at a speed safe for the lane.
 
The "caution" state allows for gracefully failure in the case that there are no
other good options available. The system typically spends most of the time in 
the cruise state.

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
 * Speed opportunity: Prefer lanes that the car could travel faster in.
 * Keep right: Stay on the right lane if possible. This weight is only slightly
 greater than the cost of changing lanes, so it will only happen if nothing else
 is going on.
 * Car avoidance: Make sure there is a safe gap between cars.
 * Lane change: Don't make unnecessary lane changes.
 
 The cost function for a canditate plan is implemented in PathPlanner::CostForTrajectory.
 
## Trajectory Generation
 
### Own car
 
Trajectory generation is done in two parts. Firstly plans for each state are
created along with s-d curves for the target lanes and speeds from the
current state. The best of these wrt to the costs above is used to generate
the final trajectory. (See: PathPlanner::PlanPath and 
PathPlanner::GeneratePlanForState)

The final trajectory is then made by taking some of the previous x-y curve 
generated along with the s-d points converted into x-y points as waypoints for 
interpolation. These interpolated points are then passed back to the simulator.
(See: PathPlanner::FinalTrajectory)

### Other cars

Other cars on the road are assumed to occupy the same lane and travel at the 
same speed. (See: PathPlanner::UpdateCarPaths)