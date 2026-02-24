# ROS1 → ROS2 conversion log (inspection_planner)

Date: 2026-02-23

Goal: Convert the provided ROS 1 folder to ROS 2 with **minimal code changes** and **preserved runtime behavior**.

---

## High-level package changes

### 1) Build system
- **ROS 1**: catkin (`catkin_package`, `rospy`, `roscpp`).
- **ROS 2**: ament (`ament_cmake` + `ament_cmake_python`).

Changes:
- Replaced `catkin` with `ament_cmake` in `package.xml`.
- Added C++ build target for `pcl_transformer`.
- Added python install via `setup.py` + `ament_python_install_package()`.

Expected behavior:
- `colcon build` builds the C++ executable and installs the Python module.
- `ros2 run inspection_planner planner_node` runs the main planner node.
- `ros2 run inspection_planner pcl_transformer` runs the pointcloud transformer.

### 2) Launch
- **ROS 1**: XML `.launch` using `<group if=...>` and `<rosparam load=...>`.
- **ROS 2**: Python launch files:
  - `launch/planner.launch.py`
  - `launch/voxel_grid_simple.launch.py`

Expected behavior:
- Launch arguments `team` and `robot` remain.
- Namespacing behavior remains (planner and transformer live under the robot namespace).
- Parameters are loaded from YAML (ROS 2 node-scoped).

### 3) Parameters YAML
- **ROS 1**: flat keys loaded into global parameter server.
- **ROS 2**: node-scoped under:
  - `planner_node: ros__parameters: ...`

Expected behavior:
- The same parameter keys continue to work in the code.
- Leading `/` in parameter reads is tolerated in code for compatibility.

---

## Python code changes

### 1) `inspection_planner/header.py`
Changes:
- Replaced `import rospy` with `import rclpy`.
- Kept message imports identical (ROS 2 message packages).
- Kept third-party helpers (`angles`, `ros_numpy`, `loguru`, `ttictoc`) for minimal behavioral drift.

Expected behavior:
- Other modules can keep using `from header import *`.
- ROS 1 APIs are no longer available; node code uses rclpy.

### 2) `inspection_planner/planner_core.py`
Changes:
- Removed all `rospy.get_param()` usage.
- Added `node` injection (`PlannerCore(node=...)`) and `_get_param()` helper.
- Replaced `rospy.Time.now()` stamps with `_now_msg()` from the node clock.
- Kept planning/math logic unchanged.

Expected behavior:
- PlannerCore still reads the same parameter names.
- "Live" parameter polling behavior is preserved where it existed (e.g., inspection distance inside `generateViewPose`).

### 3) `inspection_planner/utils.py`
Changes:
- Removed remaining `rospy.get_param()` and `rospy.Time.now()`.
- Added `PlannerUtils.set_node(node)` plus `_get_param()` and `_now_msg()` helpers.
- Treated ROS 1 private params (`~k`) by stripping `~` in `_get_param`.
- Kept the rest of the utilities unchanged.

Expected behavior:
- Utilities that previously looked up `/world_frame` etc still do.
- If no node is provided (pure python use), defaults are used.

### 4) `inspection_planner/planner_node.py`
Changes:
- Rewrote as an rclpy `Node` (`InspectionPlannerNode`).
- Replaced:
  - `rospy.Publisher` → `create_publisher`
  - `rospy.Subscriber` → `create_subscription`
  - `rospy.Service` → `create_service`
  - `rospy.ServiceProxy` → `create_client` (kept for parity)
  - `rospy.Time.now()` → `get_clock().now().to_msg()`
- Replaced ROS 1 **blocking while-loops** with a **timer-driven state machine**.

Why the timer-driven change?
- In ROS 2, blocking loops inside node constructors/callbacks can prevent the executor from servicing subscriptions.
- The timer approach preserves behavior while keeping the node responsive.

Expected behavior (matches ROS 1 intent):
- Node waits for first odom and pointcloud (soft timeout: logs a warning and continues).
- After calling `initialize_inspection` Trigger service:
  - It computes a view plan once.
  - Publishes predicted path, ref pose, cropped points, and inspection metrics.
  - Updates yaw repeatedly until thresholds are met, then replans on the next cycle.

---

## C++ code changes

### `src/pcl_transformer.cpp`
Changes:
- Ported `roscpp` node to `rclcpp`.
- Ported TF2 buffer/listener APIs to ROS 2.
- Kept the same filtering logic and transform timing.
- Preserved parameter names and defaults.

Expected behavior:
- Subscribes to `input_pointcloud`, filters (pass-through + optional voxel), transforms to `frame_id`, publishes to `output_pointcloud`.
- Uses a short TF timeout (0.05s) like the ROS 1 version.

---

## Known behavioral differences to be aware of

1) **Parameter scope**
- ROS 1 parameters were global. ROS 2 parameters are per-node.
- This port uses `allow_undeclared_parameters=True` and `automatically_declare_parameters_from_overrides=True` to behave as close to ROS 1 as possible.

2) **Launch TF publisher arguments**
- `static_transform_publisher` argument format varies slightly by distro.
- This port uses the common 8-argument Euler form: `x y z roll pitch yaw frame child`.

3) **Wait-for-message behavior**
- ROS 1 used `wait_for_message()`.
- ROS 2 port uses `spin_once()` until messages arrive (with a soft timeout).

---

## Files created in ROS 2 port
- `ros2_inspection_planner/`
  - `package.xml`
  - `CMakeLists.txt`
  - `setup.py`
  - `resource/inspection_planner`
  - `inspection_planner/` (python module)
    - `__init__.py`
    - `header.py`
    - `planner_core.py`
    - `planner_node.py`
    - `utils.py`
  - `src/pcl_transformer.cpp`
  - `launch/planner.launch.py`
  - `launch/voxel_grid_simple.launch.py`
  - `config/rosparams_exp.yaml`
  - `config/rosparams_sims.yaml`

