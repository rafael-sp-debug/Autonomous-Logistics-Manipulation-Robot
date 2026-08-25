# pick_drop_nav — Autonomous Pick, Transport & Drop-off with a Differential Robot

A ROS 2 package that drives the **Puzzlebot** differential-drive robot through a full
logistics cycle: it detects and picks up a load cube, navigates between two stations
while avoiding unknown obstacles, drops the cube inside a target zone, and returns to
its origin — fully autonomously.

The robot fuses wheel odometry with **ArUco** marker observations through an
**Extended Kalman Filter (EKF)** for drift-free localization, reaches its goals with a
**Bug2** reactive obstacle-avoidance planner, and performs the final cube alignment with
**vision-based servoing**. The whole mission is orchestrated by a finite-state machine.

The full stack was first validated in a **Gazebo digital twin** — perception, EKF and
navigation were tuned and stress-tested in simulation before a single line of code was
deployed to the physical robot.

> Built for the *Integration of Robotics and Intelligent Systems* course (TE3003B) at
> Tecnológico de Monterrey. Real-hardware tested on the Puzzlebot platform.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Digital_Twin-FF7300?logo=gazebo&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-5C3EE8?logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-blue)

---

## Demo

**Real hardware — full mission cycle**

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
                       Puzzlebot  /  Gazebo model
```

Because every node talks only through ROS 2 topics, the same executables run unchanged
against the Gazebo model and against the real Puzzlebot — only the source of
`/video_source/compressed`, `VelocityEncR/L` and `/scan` changes.

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

## Simulation-first validation (Gazebo)

Debugging a probabilistic filter on real hardware is slow and ambiguous: when the robot
misbehaves it is rarely obvious whether the fault is in the estimator, the controller, the
calibration, or the wheels slipping on the floor. To remove that ambiguity, the whole
stack was first brought up against a **Gazebo digital twin** of the Puzzlebot and the
station layout, with **RViz** used as the debugging window into the filter's internal
state (estimated pose, TF tree, covariance ellipse, LiDAR returns and goal markers).

Simulation gives access to something the real robot never provides: **ground truth**. With
the true pose available from Gazebo, the EKF estimate could be judged against a reference
instead of against intuition, which is what made it possible to characterize the wheel
noise model and tune the measurement covariances before touching the hardware.

| Subsystem | Validated in simulation |
|---|---|
| **ArUco detection + PnP** | Marker recovery across distance, viewing angle and lighting; consistency of the recovered pose axes; sanity of the range–bearing conversion |
| **EKF localization** | Growth of the covariance during dead-reckoning and its contraction when a mapped landmark enters the field of view; rejection of unmapped IDs |
| **Bug2 navigation** | Goal chaining across several waypoints, wall-following transitions, M-line re-crossing and escape from `U`-shaped traps |
| **Full integration** | The complete FSM sequence with all nodes running together, checking that no two modules publish `/cmd_vel` at once |

### ArUco detection across pose and lighting

The detector was exercised against the marker in the conditions that break monocular pose
estimation in practice: close and frontal, far away with the marker only a few pixels
wide, under strong specular glare on the floor, and at a steep oblique angle. The overlaid
axes are the pose returned by `solvePnP` — the qualitative check is that the axes stay
attached to the marker plane and do not flip as the viewpoint degrades.

<img width="443" height="299" alt="image" src="https://github.com/user-attachments/assets/3f16392b-0c2f-4c7b-bee9-b364e53810dd" />
![ArUco detection and PnP pose estimation in Gazebo under varying distance, angle and lighting](docs/media/aruco_gazebo_tests.png)

*ArUco detection in Gazebo. Top-left: near, frontal view. Top-right: long range, small
marker footprint. Bottom-left: strong floor glare. Bottom-right: oblique viewing angle.*

### Covariance collapse on landmark sighting

The clearest demonstration of what the EKF buys you: the robot drives on odometry alone
and the covariance ellipse in RViz grows steadily; the moment a mapped ArUco marker enters
the camera's field of view, the correction step fires and the ellipse snaps down around
the corrected pose.

https://github.com/user-attachments/assets/8ce27e97-5c7d-452b-afc5-70bc65ca057e

### Multi-goal navigation run

A full navigation test in the simulated environment: the robot is given a sequence of
goals and reaches them one after another, with Gazebo and RViz side by side so the
simulated world and the robot's belief about it can be compared frame by frame.



https://github.com/user-attachments/assets/402d2e5e-fdb9-42a4-b0cc-104cad033a0f



> **Note on simulation assets.** The Gazebo worlds, robot model and ArUco assets used for
> these tests are teaching materials provided by **Manchester Robotics** and are therefore
> not redistributed in this repository. The package itself is simulator-agnostic: any world
> that publishes the expected camera, encoder and LiDAR topics will run it unchanged.

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

- **Sim-to-real transfer.** The gap between the two setups is concentrated in the sensors,
  not the logic: real encoders slip and quantize, and the real camera introduces motion
  blur, exposure lag and stream dropouts that the simulated one does not. Consequently the
  parameters that needed re-tuning on hardware were the wheel noise coefficients, the
  camera calibration and the servoing gains, while the state machine, the planner and the
  filter structure carried over unchanged.

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

Simulation and hardware need **different** calibration files: the simulated camera is
ideal (no distortion, intrinsics taken straight from the sensor plugin), while the real one
must be calibrated from a checkerboard. Point `calib_file` at the right one for the setup
you are running.

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

### Running in simulation

Start the Gazebo world and the robot model first, then launch this package exactly as you
would on hardware — it subscribes to the same topic names:

```bash
# 1. Bring up the simulated world + Puzzlebot model (Manchester Robotics assets)
ros2 launch <your_gazebo_bringup> <world_launch_file>     # <-- ajusta a tu bringup

# 2. Launch the mission stack against the simulated robot
ros2 launch pick_drop_nav pick_drop_launch.py

# 3. Open RViz to watch the EKF pose, TF tree and covariance ellipse
rviz2
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

Development followed a two-stage validation: every subsystem was first exercised in the
Gazebo digital twin — where ArUco detection, EKF correction, obstacle avoidance and the
full state machine were confirmed to work together — and only then deployed to the
physical robot. The figures below are from the **real hardware** runs: three test
scenarios, five runs each. Times are full mission cycles (pick → transport → drop →
return).

| Scenario | Description | Avg. time | Success rate |
|---|---|---|---|
| **Test 1** | Logistics cycle, no obstacles | ~83.6 s | 4 / 5 |
| **Test 2** | Single box obstacle on the path | ~110 s | 4 / 5 |
| **Test 3** | `U`-shaped obstacle trap | ~136.9 s | 3 / 5 |

Obstacle avoidance worked reliably in every scenario, including the `U`-trap. The
failures were not navigation failures: they came from (a) a brief camera-stream dropout
that lost the markers, and (b) the cube being deposited right at the boundary of the
30 cm target zone rather than inside it — the precision limit of the open-loop final
approach. Both failure modes are absent in simulation, which is precisely the class of
problem a digital twin cannot catch for you.

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
- Topic-level parity between the simulated and the real robot means the same code runs in
  both, so simulation is a real development environment rather than a separate prototype.

**Limitations**
- Environment-dependent: corrections require visible markers, so poor lighting, bad marker
  placement, or calibration error degrade the whole run.
- The EKF assumes moderate error; a very wrong initial pose or a gross sensor outlier can
  diverge with no automatic recovery (mitigated, not eliminated, by ID validation).
- Reactive navigation does not anticipate obstacles, so `U`-traps cost a longer path than a
  map-based planner would.
- The cube leaves the camera's field of view in the final centimeters, forcing the
  open-loop timed approach — the most delicate part of the task.
- Simulation does not reproduce wheel slip, motion blur or camera-stream dropouts, so
  passing in Gazebo is a necessary but not a sufficient condition for passing on hardware.

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
├── docs/
│   └── media/                   # Simulation figures and demo clips
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

Simulation environment and robot model courtesy of **Manchester Robotics**.

## References

1. S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics.* MIT Press, 2005.
2. S. Garrido-Jurado et al. "Automatic generation and detection of highly reliable fiducial
   markers under occlusion." *Pattern Recognition*, 47(6), 2014.
3. R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza. *Introduction to Autonomous Mobile Robots*,
   2nd ed. MIT Press, 2011.

## License

MIT. See [LICENSE](LICENSE).
