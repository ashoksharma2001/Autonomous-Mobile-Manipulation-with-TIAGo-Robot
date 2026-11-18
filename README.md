# TIAGo Pick-and-Carry in Webots

A Webots controller that has a simulated PAL Robotics TIAGo fetch three objects from a kitchen counter, carry each to a drop-off point, and return for the next, while building a lidar occupancy grid and drawing its own path on the on-screen display.

<!-- TODO: replace with a real GIF or screenshot of your run (a 10-20 s screen recording of the Webots window works best) -->
![Demo](docs/media/demo.gif)

## Overview

| | |
|---|---|
| **Task** | Three-round pick, carry, place, and return in a kitchen scene |
| **Platform** | Webots `<R2023b>`, Python `<3.10>` |
| **Robot** | TIAGo (differential-drive base, 7-DoF arm, parallel gripper) |
| **Sensing** | GPS, compass, 2D lidar (Hokuyo URG-04LX) |
| **Navigation** | Waypoint following with a proportional heading/distance controller |
| **Mapping** | Incremental occupancy grid from lidar, shown live on the robot display |

## Quick start

```bash
git clone https://github.com/ashoksharma2001/tiago-pick-and-carry.git
cd <tiago-pick-and-carry>
pip install -r requirements.txt
```

1. Open `worlds/world.wbt` in Webots.
2. Make sure the robot's `controller` field is set to `tiago_controller`.
3. Press play. The log prints timestamped progress, ending with `Mission completed successfully`.

## Repository layout

```
.
├── controllers/
│   └── tiago_controller/
│       └── tiago_controller.py   # main controller
├── worlds/                       # Webots world file(s)
├── docs/
│   ├── walkthrough.md            # line-by-line explanation of the code
│   └── media/                    # demo GIF, screenshots
├── requirements.txt
├── LICENSE
└── README.md
```

## Method

**Navigation.** For each waypoint the controller computes distance `rho` and heading error `alpha` from GPS and compass readings, then sets left and right wheel velocities with a proportional rule. Large heading errors (over 45°) reduce the forward term so the robot turns before driving. A waypoint counts as reached within 0.5 m.

**Manipulation.** Grasping and release are scripted joint-space sequences. Each stage is verified against joint position sensors with a tolerance of 0.005 rad and a step timeout, so a stuck motion fails loudly instead of silently continuing.

**Mapping.** Each step, lidar returns (minus the 80 noisiest beams at each end) are transformed into the world frame and accumulated into a 250 × 300 grid with +0.01 per hit, saturating at 1.0. The visited path is overlaid in red.

A detailed walkthrough is in [`docs/walkthrough.md`](docs/walkthrough.md).


## Limitations

- Localization uses simulator GPS and compass, which are privileged sensors with no noise model.
- The route is fixed and open-loop; there is no obstacle avoidance or replanning.
- Grasp poses are hard-coded. The gripper force feedback is enabled but not used to confirm a grasp.
- The occupancy grid is for visualization only and is not used for decisions.

## Possible extensions

- Replace GPS/compass with lidar-based localization (e.g. scan matching or AMCL-style filtering).
- Plan on the occupancy grid (A\*, RRT\*) and add reactive obstacle avoidance.
- Use a camera and a learned or geometric grasp detector in place of fixed poses.
- Close the loop on grasp success using gripper force and finger position.

## Requirements

- [Webots](https://cyberbotics.com/) `<R2023b>`
- Python `<3.11>` and the packages in `requirements.txt`

## License

Released under the `<MIT>` License. See [`LICENSE`](LICENSE).

## Acknowledgements

Built on the TIAGo model shipped with Webots and PAL Robotics' robot design. `<Course:Introduction to Robotics with Webots ,Instructor: Nikolaus Correll>`
