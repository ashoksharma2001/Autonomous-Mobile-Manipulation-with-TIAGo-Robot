# Code Walkthrough: TIAGo Pick-and-Carry Controller

This document explains how [`tiago_controller.py`](../controllers/tiago_controller/tiago_controller.py) makes a simulated TIAGo robot in Webots fetch three objects, carry each to a drop-off point, and return for the next one, while building a lidar map and drawing its own path on screen. It follows the order the code is organised in: configuration, setup, perception, driving, arm motion, and finally the mission itself.

[← Back to README](../README.md)

## Contents

1. [The big picture](#1-the-big-picture)
2. [Imports](#2-imports)
3. [Configuration constants](#3-configuration-constants)
4. [Device setup](#4-device-setup)
5. [Knowing where it is](#5-knowing-where-it-is)
6. [Building the occupancy map](#6-building-the-occupancy-map)
7. [Drawing the path](#7-drawing-the-path)
8. [Driving to waypoints](#8-driving-to-waypoints)
9. [Moving the arm](#9-moving-the-arm)
10. [Picking up and putting down](#10-picking-up-and-putting-down)
11. [The mission](#11-the-mission)
12. [Limitations and ideas for extension](#12-limitations-and-ideas-for-extension)
13. [Summary](#13-summary)

---

## 1. The big picture

The controller has two jobs that run together:

- **Moving through the world.** The base is steered toward a list of (x, y) waypoints using GPS and compass readings.
- **Handling objects.** The arm and gripper run short, scripted pose sequences to pick something up and set it down.

While the base is driving, every simulation step also folds the latest lidar sweep into an occupancy grid and redraws the robot's trail on the Webots display. Everything lives in one class, `TiagoMission`, with the tunable numbers and route data kept at module level so they can be changed without touching the logic.

```mermaid
flowchart LR
    A[Ready pose] --> B[Drive to counter]
    B --> C[Pick up]
    C --> D[Drive to drop-off]
    D --> E[Put down]
    E --> F[Drive room loop and return]
    F -->|rounds 2 and 3| C
    F -->|after round 3| G[Done]
```

## 2. Imports

Only two third-party modules are needed:

- `controller.Supervisor` is the Webots API. A Supervisor is a robot controller that can also reach into the scene tree, which is how the script moves a visual marker object around.
- `numpy` handles the lidar geometry.

`datetime` is used for timestamped log lines.

## 3. Configuration constants

The constants at the top of the file fall into four groups.

**Driving.** `WHEEL_MAX_SPEED` caps wheel velocity at 6.28 rad/s, roughly one wheel revolution per second. `ARRIVAL_RADIUS` (0.5 m) decides when a waypoint counts as reached. `STEER_GAIN` and `DRIVE_GAIN` are the proportional gains for turning and forward motion, and `SHARP_TURN_LIMIT` (45 degrees) marks where the controller switches to its gentler forward push.

**Mapping.** The occupancy grid is 250 by 300 cells. The lidar has 667 beams spanning about ±120 degrees, and `LIDAR_TRIM` discards 80 beams at each end because the outermost readings are noisy.

**Arm.** A timeout of 1000 simulation steps and a joint tolerance of 0.005 rad define when an arm move is considered finished or failed.

**Poses and route data.** `HOME_POSE` lists every controllable joint with a resting value, and doubles as the default pose for arm moves. `READY_POSE` is the folded-up, gripper-open pose used at the start. The route is stored as data: a counter position, a pass-through point, a per-round table of drop-off and return spots, and a loop of waypoints around the room.

## 4. Device setup

### Actuators

Each joint name in `HOME_POSE` is looked up as a Webots motor and stored in a dictionary. The two wheel motors are switched to velocity control by setting their target position to infinity, after which only `setVelocity` is used. Force feedback is switched on for both gripper fingers so the simulator reports grip force.

### Sensors

GPS, compass and lidar are enabled at the simulation time step, and the lidar also produces a point cloud. The script then fetches a scene object named `marker` and keeps a handle on its `translation` field; during driving the marker is moved onto the waypoint currently being chased, which makes the route visible in the 3D view.

Finally, a position sensor is enabled for every joint so arm moves can be verified. Sensor names follow the pattern `<joint>_sensor`, except for the gripper fingers, whose sensors are named by inserting `sensor_` before `finger_joint`. A missing sensor prints a warning rather than aborting.

## 5. Knowing where it is

`_pose` returns the robot's world position from the GPS and its heading from the compass. The heading is `atan2(compass_x, compass_y)`, which converts the compass vector into an angle in the range −π to π.

`world_to_grid` turns world metres into grid indices with a fixed scale of 52 cells per metre plus offsets that line the grid up with the simulated room (124.8 for x, 93.834 for y, with y flipped because image rows grow downward). Results are clamped to the grid bounds so a stray reading can never index outside the array.

## 6. Building the occupancy map

`_integrate_scan` runs once per driving step:

1. Read the lidar ranges and replace any infinite value (no return) with 100 so the maths stays finite.
2. Drop the trimmed beams at both ends, along with their angles.
3. Convert each beam from polar to Cartesian form in the robot's frame, as homogeneous points (x, y, 1).
4. Multiply by a 3×3 transform built from the robot's heading and position to move all points into world coordinates.
5. For each point, find its grid cell, raise that cell's value by 0.01 (never above 1.0), and paint the pixel on the display in a grey level proportional to the value.

The small increment means a cell must be hit repeatedly before it looks solid, which makes the map tolerant of occasional bad readings. Because points beyond the sensor's range are set to 100 m they land on the clamped border, so they mostly affect the edge of the grid.

## 7. Drawing the path

`_paint_path` stores the robot's current grid cell in a set and then redraws every stored cell in red. Redrawing is needed because the map painting can overwrite trail pixels with grey. Using a set means only cells actually visited are drawn, instead of scanning the entire grid each step.

## 8. Driving to waypoints

### The control law

For the waypoint being chased, `drive` computes the straight-line distance and the heading error, which is the bearing to the waypoint minus the robot's heading, wrapped back into the range −π to π.

`_wheel_speeds` converts those two numbers into wheel velocities with a simple proportional rule:

- left = −(heading error × turn gain) + (distance × push gain)
- right = +(heading error × turn gain) + (distance × push gain)

A positive heading error therefore slows the left wheel and speeds up the right, turning the robot toward the target, while the distance term drives both wheels forward. If the heading error is larger than 45 degrees, both gains are scaled down (the turn gain by half and the push gain by an eighth), so the robot mainly rotates in place before moving ahead. Both outputs are clamped to the wheel speed limit.

### Advancing through the list

When the distance falls below the arrival radius, the index moves on to the next waypoint. After the last one, the wheels are stopped and the function returns `True`. Otherwise the computed speeds are applied, the scan is integrated, and the path is repainted before the next simulation step.

Any exception stops the wheels first and is then re-raised as a `RuntimeError` naming the action, so a failure never leaves the robot rolling.

## 9. Moving the arm

`move_arm` sends a target position to each joint in a dictionary and then steps the simulation until all of those joints are within tolerance according to their position sensors. Joints without a sensor are skipped in the check. If 1000 steps pass first, a timeout error is raised. Unknown joint names are rejected up front.

`pause` simply advances the simulation for roughly the requested number of milliseconds by stepping `ms / time_step` times. The scripted sequences use it to let the arm settle between stages.

## 10. Picking up and putting down

Both sequences are written as a short list of stages, each with a label, a joint pose and an error message, and one loop runs them in order with a 300 ms pause after each.

**Pick up**

1. Straighten the shoulder and elbow and rotate the forearm, extending the hand horizontally toward the object.
2. Close the fingers to 0.04.
3. Fold the arm back up while still holding the object.

**Put down**

1. Lower the torso, bring the arm out, and tilt the wrist.
2. Open the fingers to 0.07 to release the object.
3. Raise the torso and tuck the arm back into its carrying pose.

If any stage fails, the error is wrapped with the sequence number so the log shows exactly which round broke.

## 11. The mission

`run` ties it all together:

1. Move the arm to the ready pose.
2. Drive to the counter position in front of the objects.
3. For rounds 1 to 3: pick up, drive through the pass-through point to that round's drop-off spot, put down, then drive the room loop and finish at that round's return spot.

The return spots and drop-off spots differ for each round on purpose, so the objects are not placed on top of one another and the robot does not collide with items it has already moved. Success logs a completion message and returns `True`; any failure logs the reason, stops the wheels and returns `False`.

The file ends with a standard `__main__` guard that builds a `TiagoMission` and calls `run`.

## 12. Limitations and ideas for extension

- **No obstacle avoidance.** The route is fixed. A local planner using the lidar, or a planner such as A* or RRT on the occupancy grid, would let the robot cope with unexpected obstacles.
- **Simple proportional steering.** Adding derivative or integral terms, or a smoother path through the waypoints, would reduce overshoot and jerky turns.
- **Perception limited to GPS and compass.** Fusing wheel odometry or an IMU with a Kalman filter would be closer to what a real robot needs, since GPS is a simulation convenience.
- **Open-loop grasping.** Objects are grabbed at hard-coded poses. Using the gripper force feedback or a camera to confirm a grasp would make it robust to position errors.
- **Map is only for display.** The occupancy grid is never used for decisions; thresholding or inflating it would be the first step toward planning with it.

## 13. Summary

The controller is small but covers the main parts of a mobile manipulation stack: sensing, a proportional navigation loop, scripted arm behaviour with verification and timeouts, and a lightweight mapping layer, all with defensive error handling that stops the robot safely when something goes wrong. Because the routes and poses are data rather than code, adapting it to a new scene mostly means editing the tables at the top of the file.
