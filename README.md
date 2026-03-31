# odometry_motion_model — ROS 2 Humble

Probabilistic odometry motion model node that subscribes to `/odom`
and publishes a particle cloud, estimated pose, and trajectory paths
for visualization in RViz2.

Implements the three building blocks from Probabilistic Robotics Ch. 5.4:
- **inverse_motion_model** — odometry primitives (δrot1, δtrans, δrot2)
- **motion_model**         — likelihood p(x_t | u_t, x_{t-1})
- **sample_motion_model**  — particle propagation via CLT sampling

---

## Prerequisites

```bash
sudo apt install ros-humble-tf-transformations python3-transforms3d
```

---

## Build

```bash
# Copy this package into your workspace
cp -r odometry_motion_model ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select odometry_motion_model
source install/setup.bash
```

---

## Run

### Option A — launch file (recommended)

```bash
# Default params, no RViz2
ros2 launch odometry_motion_model motion_model.launch.py

# Open RViz2 automatically
ros2 launch odometry_motion_model motion_model.launch.py with_rviz:=true

# Higher noise, more particles
ros2 launch odometry_motion_model motion_model.launch.py \
    alpha1:=0.5 alpha2:=0.5 alpha3:=0.05 alpha4:=0.05 \
    num_samples:=2000

# Enable p3 marginalisation
ros2 launch odometry_motion_model motion_model.launch.py marginalise_p3:=true
```

### Option B — run node directly

```bash
ros2 run odometry_motion_model odometry_motion_model_node \
    --ros-args \
    -p alpha1:=0.1 -p alpha2:=0.1 \
    -p alpha3:=0.01 -p alpha4:=0.01 \
    -p num_samples:=1000 \
    -p marginalise_p3:=false \
    -p fixed_frame:=odom
```

### Option C — RViz2 only (after node is already running)

```bash
rviz2 -d $(ros2 pkg prefix odometry_motion_model)/share/odometry_motion_model/rviz/motion_model.rviz
```

---

## Published topics

| Topic             | Type                            | Description                     |
|-------------------|---------------------------------|---------------------------------|
| `/estimated_pose` | `geometry_msgs/PoseStamped`     | Circular mean of particle cloud |
| `/particle_cloud` | `geometry_msgs/PoseArray`       | All N sampled particles         |
| `/odom_path`      | `nav_msgs/Path`                 | Ground-truth odometry trail     |
| `/particle_path`  | `nav_msgs/Path`                 | Estimated pose trail            |

---

## Parameters

| Parameter       | Default | Description                               |
|-----------------|---------|-------------------------------------------|
| `alpha1`        | 0.1     | Rotation noise from rotation              |
| `alpha2`        | 0.1     | Rotation noise from translation           |
| `alpha3`        | 0.01    | Translation noise from translation        |
| `alpha4`        | 0.01    | Translation noise from rotation           |
| `num_samples`   | 1000    | Number of particles                       |
| `marginalise_p3`| false   | Drop p3 term (ignore final rotation prob) |
| `fixed_frame`   | odom    | RViz2 fixed frame                         |

---

## RViz2 display guide

| Display            | Color       | What you see                                |
|--------------------|-------------|---------------------------------------------|
| Grid               | gray        | Ground plane                                |
| Odometry GT Path   | blue        | True path from raw `/odom`                  |
| Estimated Pose Path| orange      | Path of particle mean (drifts with noise)   |
| Particle Cloud     | red arrows  | 1000 sampled poses (spread = uncertainty)   |
| Estimated Pose     | red arrow   | Current best estimate                       |

Increase `alpha1`/`alpha2` to see the cloud spread widen on turns.
Set `marginalise_p3:=true` to see how the cloud shape changes when
the final rotation uncertainty is ignored.
