# Motion Planning Interface (ROS 2 + MoveIt 2)

<img width="3030" height="1722" alt="image" src="https://github.com/user-attachments/assets/ff821583-6ee6-40ed-ae18-49aab35b771b" />

**Authors:** Derek Dietz, Theodore Coulson, Andnet DeBoer

A lightweight Python interface for **MoveIt 2** that simplifies motion planning, kinematics, and planning scene management for the **Franka Panda** robot. The goal is to abstract away low‑level MoveIt actions/services and provide a clean, easy‑to‑use API.

---

## Features

### **Motion Planning**

* Joint-space planning
* Pose planning (position/orientation/both)
* Cartesian path planning
* Optional immediate trajectory execution
* Save/load planned trajectories

### **Robot State**

* Forward & inverse kinematics
* Retrieve current joint state and end-effector pose

### **Planning Scene**

* Add/remove collision objects
* Attach/detach objects to/from the end‑effector
* Load preset scenes from parameters

### **Integration Layer**

* Unified `MotionPlanningInterface` combining planner, scene, and robot state
* Compatible with any user-defined ROS 2 node

---

## Pick-and-Place Demo

A sample node `pick_node` demonstrates:

1. Move above object
2. Open gripper
3. Lower onto object
4. Close gripper
5. Lift
6. Attach collision object
7. Move around obstacle
8. Release & detach

The package includes a launch file `pickplace.launch.py` that runs the full demo.

---

## Quickstart

```bash
colcon build --symlink-install
source install/setup.bash
```

Launch RViz + Franka demo:

```bash
ros2 launch motion_planning pickplace.launch.py demo:=true
```

---

## Repository Structure

```
homework-3-part-2-actual-north-western-northwestern/
├── README.md
├── motion_planning/          # Main ROS 2 package
│   ├── launch/               # Python launch file
│   ├── config/               # Config files
│   ├── motion_planning/      # Python node
│   └── tests/                # Integration test
```

---

## Testing

```bash
colcon test
colcon test-result --verbose
```

Includes:

* Integration tests

---

## Documentation

```bash
rosdoc2 build .
```

---

## Video

[![alt text](image-1.png)](https://github.com/user-attachments/assets/a237eb4f-70be-4c88-b44f-16cb0bdadd59)



https://github.com/user-attachments/assets/a7bc60b8-02bf-4436-83d7-839d285dcf5b



