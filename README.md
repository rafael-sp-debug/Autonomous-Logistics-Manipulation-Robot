# pick_drop_nav — Autonomous Pick, Transport & Drop-off with a Differential Robot

A ROS 2 package that drives the **Puzzlebot** differential-drive robot through a full
logistics cycle: it detects and picks up a load cube, navigates between two stations
while avoiding unknown obstacles, drops the cube inside a target zone, and returns to
its origin — fully autonomously.

The robot fuses wheel odometry with **ArUco** marker observations through an
**Extended Kalman Filter (EKF)** for drift-free localization, reaches its goals with a
**Bug2** reactive obstacle-avoidance planner, and performs the final cube alignment with
**vision-based servoing**. The whole mission is orchestrated by a finite-state machine.

> Built for the *Integration of Robotics and Intelligent Systems* course (TE3003B) at
> Tecnológico de Monterrey. Real-hardware tested on the Puzzlebot platform.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-5C3EE8?logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-blue)

---

## Demo

https://github.com/user-attachments/assets/05913622-7189-42b9-b528-254710c863b0



|  | |
|---|---|
| **Task** | Pick a cube at the load station, avoid obstacles, drop it at the unload station, return to origin |
| **Stations distance** | 2 m apart, each marked with a 30 cm tape square |
| **Sensors** | Monocular camera (ArUco), wheel encoders, 360° LiDAR |
| **Localization** | EKF fusing odometry + ArUco landmarks |
| **Navigation** | Bug2 reactive planner (handles `U`-shaped traps) |
| **Manipulation** | Servo-driven forklift, vision-guided final approach |

---

## How it works

The system is split into independent nodes so vision, control, and localization can be
tested in isolation and then composed. Data flows from perception → localization →
navigation/servoing, all sequenced by the mission coordinator.

```
                 /video_source/compressed
                          │
                          ▼
                ┌───────────────────┐  /aruco_target (cube pose)
                │  aruco_detector   │──────────────────────────────┐
                │  (vision / PnP)   │  /aruco_detections (landmarks)│
                └───────────────────┘──────────┐                   │
                                                ▼                   ▼
   VelocityEncR/L ──►  ┌───────────────────┐         ┌──────────────────────┐
                       │   localisation    │  /odom  │  center_and_approach │
                       │   (EKF)           │────┬────►│  (visual servoing +  │
                       └───────────────────┘    │    │   forklift servo)    │
                                                 │    └──────────────────────┘
                                                 │        │  /ca_status  ▲
                                                 ▼        ▼              │ /mission_state
   /scan (LiDAR) ──►  ┌───────────────────┐   ┌──────────────────────────┴───┐
                      │   bug2 / bug0     │◄──│      mission_coordinator      │
                      │  (Bug2 planner)   │   │   (finite-state machine)      │
                      └─────────┬─────────┘   └──────────────────────────────┘
                                │ /cmd_vel                /target, /ServoAngle
                                ▼
                            Puzzlebot
```

### Nodes

| Node (executable) | Role | Subscribes | Publishes |
|---|---|---|---|
| **`aruco_detector_node`** | Vision system. Undistorts the camera image, detects ArUco markers, and recovers each marker's pose with `solvePnP`. Wall/landmark markers are sent as range–bearing measurements; the cube marker is sent with its yaw misalignment for servoing. | `/video_source/compressed` | `/aruco_detections`, `/aruco_target` |
| **`localisation`** | EKF localization. Predicts the pose by dead-reckoning the differential-drive model and corrects it with landmark observations whose IDs are in a known map. Publishes the corrected pose and TF, with the covariance ellipse for RViz. | `VelocityEncR`, `VelocityEncL`, `/aruco_detections` | `/odom`, TF `world → base_footprint` |
| **`bug2`** (or **`bug0`**) | Reactive navigation. Drives to a goal on the *M-line*; when the LiDAR detects an obstacle it follows the wall until it can rejoin the M-line closer to the goal. Only active during navigation states. | `/odom`, `/target`, `/scan`, `/mission_state` | `/cmd_vel` |
| **`center_and_approach`** | Vision-guided manipulation. Takes over near a station: centers on the cube marker, approaches, aligns yaw, then runs a timed forklift routine (lower → advance → raise → reverse) to pick or deposit. | `/aruco_target`, `/mission_state` | `/cmd_vel`, `/ServoAngle`, `/ca_status` |
| **`mission_coordinator`** | The brain. A finite-state machine that sequences the whole mission: navigate to pickup → pick → navigate to dropoff → deposit → return to origin. | `/odom`, `/ca_status`, `/aruco_target` | `/target`, `/mission_state`, `/ServoAngle` |

### Mission state machine (`mission_coordinator`)

```
IDLE ─► NAVIGATE_TO_PICKUP ─► PICK ─► NAVIGATE_TO_DROPOFF ─► DEPOSIT ─► NAVIGATE_TO_ORIGIN ─► DONE
            │  cube seen +        │  ca_status          │  reached      │  servo settled    │  reached
            │  within trigger     │  == 'done'          │  dropoff      │                   │  origin
```

`mission_state` is broadcast on every tick so the navigation and servoing nodes know
whether they should be active — this prevents two modules from sending conflicting
velocity commands at the same time.

---

## Technical approach

- **Localization (EKF).** Odometry alone drifts: each integration step accumulates wheel
  slip, encoder quantization, and discretization error. The EKF runs a periodic
  *prediction* (propagating pose and growing covariance through the motion Jacobian) and
  an event-driven *correction* every time a mapped ArUco landmark is observed, which
  contracts the covariance. The wheel noise is characterized experimentally
  (`σ² = a·|v| + b` per wheel) rather than guessed, which is what makes the filter behave
  consistently. The Joseph form is used for the covariance update for numerical stability.

- **Vision (ArUco + PnP).** Markers are detected from the `DICT_4X4_1000` dictionary.
  The camera image is undistorted using the calibration YAML, then `solvePnP` recovers
  each marker's 3D pose. Camera-frame coordinates are transformed to the robot frame
  (`x_robot = z_cam + 0.1241`, `y_robot = -x_cam`) and expressed as range–bearing for
  control. The cube marker additionally yields a yaw misalignment used to square up before
  pickup.

- **Navigation (Bug2).** The planner needs no prior map. It defines the M-line from start
  to goal and switches between *go-to-goal* and *follow-wall*, leaving an obstacle only
  when it re-crosses the M-line strictly closer to the goal. The 360° LiDAR is split into
  six sectors (front, front-left, left, back, right, front-right) for reactive wall
  following. This completeness guarantee is what lets the robot escape `U`-shaped traps
  where greedier methods get stuck.

- **Manipulation.** Because the cube marker leaves the camera's field of view in the last
  few centimeters of the approach, the final pickup/deposit is run open-loop as a timed
  sequence once the robot is centered and aligned — a deliberate design choice to handle
  the sensor's blind spot.

---

## Installation

This package targets **ROS 2 (Humble)** with `ament_python`.

```bash
# 1. Clone into your workspace
cd ~/ros2_ws/src
git clone <your-repo-url> pick_drop_nav

# 2. Install dependencies (from the workspace root)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build and source
colcon build --packages-select pick_drop_nav
source install/setup.bash
```

**Key dependencies:** `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`,
`tf_transformations`, `cv_bridge`, `python3-opencv`, `python3-numpy`.

### Camera calibration

The vision node expects a calibration file at `~/.ros/camera_info/puzz_cam.yaml`
(camera matrix + distortion coefficients). If it is missing, the node falls back to a
generic matrix and warns you — pose estimates will be inaccurate, so calibrate first.

---

## Usage

Launch the full mission with the provided launch file:

```bash
ros2 launch pick_drop_nav pick_drop_launch.py
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `pickup_x`, `pickup_y` | `0.0`, `1.25` | Load-station coordinates (m) |
| `dropoff_x`, `dropoff_y` | `0.0`, `-1.20` | Unload-station coordinates (m) |
| `bug_algorithm` | `bug2` | Obstacle-avoidance planner: `bug0` or `bug2` |
| `calib_file` | `~/.ros/camera_info/puzz_cam.yaml` | Camera calibration YAML |
| `target_id` | `17` | ArUco ID glued to the load cube |

Example with a custom layout and the Bug0 planner:

```bash
ros2 launch pick_drop_nav pick_drop_launch.py \
    pickup_x:=0.0 pickup_y:=1.0 dropoff_x:=0.0 dropoff_y:=-1.0 \
    bug_algorithm:=bug0 target_id:=17
```

### Running nodes individually

Useful for debugging a single subsystem:

```bash
ros2 run pick_drop_nav aruco_detector_node
ros2 run pick_drop_nav localisation
ros2 run pick_drop_nav bug2            # or bug0
ros2 run pick_drop_nav center_and_approach
ros2 run pick_drop_nav mission_coordinator
```

---

## Results

The system was validated on real hardware across three test scenarios, five runs each.
Times are full mission cycles (pick → transport → drop → return).

| Scenario | Description | Avg. time | Success rate |
|---|---|---|---|
| **Test 1** | Logistics cycle, no obstacles | ~83.6 s | 4 / 5 |
| **Test 2** | Single box obstacle on the path | ~110 s | 4 / 5 |
| **Test 3** | `U`-shaped obstacle trap | ~136.9 s | 3 / 5 |

Obstacle avoidance worked reliably in every scenario, including the `U`-trap. The
failures were not navigation failures: they came from (a) a brief camera-stream dropout
that lost the markers, and (b) the cube being deposited right at the boundary of the
30 cm target zone rather than inside it — the precision limit of the open-loop final
approach.

---

## Strengths & limitations

**Strengths**
- The EKF corrects odometry drift at very low computational cost — a 3-state filter runs
  comfortably in real time on the Puzzlebot, unlike heavier particle filters.
- Unique ArUco IDs turn a cheap monocular camera into an absolute position sensor with no
  data-association ambiguity.
- Separating prediction from correction makes the robot tolerant to short vision dropouts:
  it keeps moving on odometry and snaps back when a marker reappears.
- Bug2's completeness guarantee escapes complex obstacles where other reactive methods get
  trapped.
- A single explicit state machine prevents conflicting velocity commands across modules.

**Limitations**
- Environment-dependent: corrections require visible markers, so poor lighting, bad marker
  placement, or calibration error degrade the whole run.
- The EKF assumes moderate error; a very wrong initial pose or a gross sensor outlier can
  diverge with no automatic recovery (mitigated, not eliminated, by ID validation).
- Reactive navigation does not anticipate obstacles, so `U`-traps cost a longer path than a
  map-based planner would.
- The cube leaves the camera's field of view in the final centimeters, forcing the
  open-loop timed approach — the most delicate part of the task.

---

## Project structure

```
pick_drop_nav/
├── pick_drop_nav/
│   ├── aruco_detector.py        # Vision: ArUco detection + PnP pose
│   ├── localisation.py          # EKF localization (odometry + landmarks)
│   ├── bug2.py / bug0.py        # Reactive obstacle-avoidance planners
│   ├── center_and_approach.py   # Vision servoing + forklift routine
│   └── main_controller.py       # Mission finite-state machine
├── launch/
│   └── pick_drop_launch.py      # Full-mission launch file
├── package.xml
├── setup.py
└── README.md
```

---

## Authors

- **Leyberth Jaaziel Castillo Guerra**
- **Rafael Soto Padilla**

Developed for the *Integration of Robotics and Intelligent Systems* (TE3003B) final
challenge, Tecnológico de Monterrey — Campus Estado de México.

## References

1. S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics.* MIT Press, 2005.
2. S. Garrido-Jurado et al. "Automatic generation and detection of highly reliable fiducial
   markers under occlusion." *Pattern Recognition*, 47(6), 2014.
3. R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza. *Introduction to Autonomous Mobile Robots*,
   2nd ed. MIT Press, 2011.

## License

MIT. See [LICENSE](LICENSE).
