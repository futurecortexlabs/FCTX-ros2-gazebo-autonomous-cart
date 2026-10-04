# FCTX-ROS 2 Autonomous Cart Lab

<p align="center"><b>English</b> | <a href="README.ja.md">日本語</a></p>

**A four-wheeled cart simulation with obstacle avoidance, SLAM, localization, and autonomous delivery.**

Built with ROS 2 Jazzy and Gazebo Harmonic, this lab lets you drive the cart manually, build a map, navigate to a goal, and deliver to multiple locations. The Japanese-language GUI provides seven courses and three pose modes.

Control, the GUI, safety monitoring, and process management are implemented in Python and connected through ROS 2. The system uses `slam_toolbox` for SLAM, `nav2_amcl` for localization, and `robot_localization` to fuse IMU and wheel measurements. Path planning, tracking, and delivery management are implemented in this repository.

![FCTX control panel: saved warehouse map and estimated cart pose](docs/images/control-panel.png)

## Features

| Feature | Description |
|---|---|
| Manual driving | WASD, arrow keys, and on-screen buttons. The cart moves while a key is held; losing window focus clears the input. |
| Free driving | Compare candidate trajectories using 360° LiDAR and move around obstacles without a fixed lap direction. |
| Goal navigation | Select a destination on the map, plan with A*, and stop on arrival. |
| Delivery missions | Visit multiple destinations with wait times. Save, load, pause, and cancel routes. |
| Additional obstacles | Replan around LiDAR-detected obstacles. Stop and wait when no route is available. |
| SLAM and localization | Build and save occupancy maps with LiDAR, then use the saved maps with AMCL after restarting. |
| Safety monitoring | Monitor stale commands, sensors, and pose data, obstacle proximity, contacts, and the stop lock. |
| Operator controls | Select among seven courses, adjust speed, click the map, set an initial pose estimate, and manage startup and shutdown. |

The add/remove box buttons are available in `simulation` mode. LiDAR obstacle detection and replanning also operate in SLAM and AMCL modes.

## Quick start

Validation was performed on **Windows + WSL2 / Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**. Follow the [setup guide](docs/SETUP.md) first to install ROS 2 and the required packages.

```bash
git clone https://github.com/futurecortexlabs/FCTX-ros2-gazebo-autonomous-cart.git
cd FCTX-ros2-gazebo-autonomous-cart
bash run_courses.sh --course warehouse_course.sdf --pose-source localization --start
```

When the GUI shows “読み込み済み” (loaded), select a traversable location on the map at the right and press “この目的地へ移動” (move to this destination). The cart starts stopped in manual mode. To try a delivery mission, load `routes/warehouse_course_coverage_delivery.json` from “配送ミッション” (delivery mission).

[Startup and recovery](RUNBOOK.md) / [Goal navigation](GOAL_NAVIGATION.md) / [Delivery missions](MISSION_GUIDE.md)

## Courses and pose modes

| Course | Layout |
|---|---|
| Circle | Basic loop enclosed by inner and outer walls |
| Obstacle | Circle course with two boxes and one pole |
| Oval | Loop with near-straight sections and curves |
| Warehouse | Open area with outer walls and four shelves |
| Large oval | 24 × 16 m loop |
| Large warehouse | 26 × 20 m course with nine shelves |
| Slalom | 24 × 16 m course with three alternating walls to navigate around |

| Pose mode | Inputs for pose and path planning | Main use |
|---|---|---|
| `simulation` | Gazebo ground-truth pose and known SDF geometry | Control, GUI, and obstacle-avoidance experiments |
| `slam` | EKF odometry, SLAM pose estimate, and observed occupancy map | Mapping and navigation using the generated map |
| `localization` | EKF odometry, AMCL pose estimate, and saved occupancy map | Delivery using a saved map |

Control, safety gating, and localization in SLAM/AMCL modes do not subscribe to ground-truth pose. Ground truth is used for external accuracy validation and display. Observed maps of the full circuit and main passages, together with `routes/*_coverage_delivery.json`, are included for all seven courses. Short return-trip samples are also retained. Planning through unknown cells is disallowed; unobserved areas must be mapped with SLAM before use.

## Engineering decisions

- **Skid during four-wheel turns:** fuse wheel speed and IMU orientation/angular velocity with an EKF, rather than relying on raw wheel-derived yaw.
- **Safe motion commands:** an independent SafetyGate determines the final velocity and outputs zero when required data is stale.
- **Map and route consistency:** verify saved maps against a course hash, and distinguish `world` and `map` route coordinate frames.
- **Changes during navigation:** retain additional obstacles detected by LiDAR and incorporate new observations into replanning.
- **Reliable startup and shutdown:** check pose and map readiness, retry once only during startup, verify that Gazebo is paused before terminating owned processes, and handle shutdown immediately after startup or during a course change.
- **Processing cost:** batch distance calculations with NumPy and reuse distance fields and GUI images when map content is unchanged.

See the [architecture guide](docs/ARCHITECTURE.md) for message flow, algorithms, and the roles of the main files.

## Validation

The **2026-10-03 finishing validation** used maps and delivery routes covering the full circuits and main passages.

| Test | Result |
|---|---|
| Observed coverage of the seven included maps | 100% of the spawn-connected safe reference region observed; traversable map area 86.7–97.4% |
| Full SLAM tours with the final common configuration | Seven courses, 105 inspection locations, zero contacts, maximum return-position estimation error 1.95 cm, clean shutdown |
| Full AMCL tours on seven courses | 98 destinations, zero contacts, all deliveries completed and stopped |
| Estimated versus ground-truth pose at return | AMCL position difference 0.63–4.52 cm |
| Continuous warehouse delivery with AMCL | 30 min 51 s, 11 tours, 132 destinations, zero contacts |
| Shutdown and course switching | Nine AMCL/simulation cases, seven SLAM cases, and four final-GUI cases recorded with their source versions |
| Additional obstacles and pose-update interruption | Replanning and arrival verified; safe stop and recovery verified when AMCL was suspended |
| Static checks and first-run diagnostics | 17 unit-test entry points, six runner cases, and dependency/seven-map diagnostics in a separate copy passed |
| Rendering | Delivery and clean shutdown verified with WSL rendering and llvmpipe on the same machine |

The observed-coverage denominator is the spawn-connected reference region after accounting for the cart radius and clearance margin. Isolated regions enclosed by inner walls are excluded. Traversable map area is a separate metric after inflating obstacles and unknown cells. Position error measures the difference between the estimated and ground-truth pose at return; it is distinct from the remaining distance to the goal. See [validation and scope](docs/VALIDATION.md) for conditions, SLAM configuration comparisons, result JSON files, and reproduction steps.

## Repository layout

```text
launch/       Launch Gazebo, bridges, control, and SLAM/AMCL together
scripts/      Control, GUI, path planning, map saving, diagnostics, validation
config/       EKF, SLAM, and AMCL configuration
worlds/       SDF files for the four-wheeled cart, sensors, and seven courses
maps/         Course-specific occupancy maps and consistency metadata
routes/       Delivery samples with explicit coordinate frames
fonts/        Japanese font configuration
validation/   Saved validation results organized by date
docs/         Setup, architecture, validation, and screenshots
tests/        Reference data for validation, including earlier implementations
```

Runtime logs, PID files, and screenshots are written to `logs/` and are excluded from Git. Windows font files are not included.

## Documentation

The GUI and the detailed guides below are currently in Japanese.

| Topic | Guide |
|---|---|
| Dependencies and first launch | [Setup](docs/SETUP.md) |
| Startup, shutdown, and troubleshooting | [RUNBOOK.md](RUNBOOK.md) |
| Architecture, communication, and main files | [Architecture](docs/ARCHITECTURE.md) |
| Course selection, manual driving, and speed | [COURSES.md](COURSES.md) / [MANUAL_MODE.md](MANUAL_MODE.md) |
| Goals, delivery, and additional obstacles | [GOAL_NAVIGATION.md](GOAL_NAVIGATION.md) / [MISSION_GUIDE.md](MISSION_GUIDE.md) / [DYNAMIC_OBSTACLES.md](DYNAMIC_OBSTACLES.md) |
| SLAM and localization | [SLAM_LOCALIZATION.md](SLAM_LOCALIZATION.md) |
| Test conditions, results, and reproduction | [Validation and scope](docs/VALIDATION.md) / [ACCEPTANCE.md](ACCEPTANCE.md) |
| Optimization and measurement conditions | [OPTIMIZATION.md](OPTIMIZATION.md) |
