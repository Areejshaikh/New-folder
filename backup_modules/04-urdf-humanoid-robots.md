---
title: "URDF for Humanoid Robots: Unified Robot Description Format"
sidebar_label: "URDF for Humanoids"
description: "Comprehensive guide to creating Unified Robot Description Format files for humanoid robots in ROS 2"
---

# URDF for Humanoid Robots: Unified Robot Description Format

## Overview

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF plays a crucial role in defining the physical structure, kinematic properties, and visual/physical representations that can be used in simulation and real-world robot control.

This module provides a comprehensive guide to creating URDF files for humanoid robots, focusing on applications relevant to the PAHR curriculum.

## Learning Objectives

By the end of this module, you will be able to:

1. Create a basic humanoid robot URDF model with proper kinematic chain
2. Define physical properties (inertia, geometry, visual representation)
3. Specify joint limits and ranges of motion appropriate for humanoid robots
4. Create materials and visual properties for realistic rendering
5. Validate URDF files for correctness
6. Integrate URDF models with simulation environments (Gazebo)

## Prerequisites

- Basic understanding of XML syntax
- Knowledge of robotic kinematics (covered in Module 1)
- Understanding of ROS 2 concepts (nodes, topics)
- Familiarity with 3D coordinate systems and transformations

## Table of Contents

1. [XML Basics for URDF](#xml-basics-for-urdf)
2. [URDF Elements and Structure](#urdf-elements-and-structure)
3. [Designing Humanoid Kinematics](#designing-humanoid-kinematics)
4. [Creating URDF for Humanoid Robots](#creating-urdf-for-humanoid-robots)
5. [Physical Properties in URDF](#physical-properties-in-urdf)
6. [Visual Properties and Materials](#visual-properties-and-materials)
7. [Validation and Testing](#validation-and-testing)
8. [Integration with Simulation](#integration-with-simulation)
9. [Advanced Topics](#advanced-topics)
10. [Exercises](#exercises)

---

## XML Basics for URDF

URDF is expressed in XML format, so we'll start with a brief review of XML concepts relevant to URDF:

### Basic XML Structure
```xml
<?xml version="1.0" ?>
<robot name="my_robot">
  <!-- Robot elements go here -->
</robot>
```

### Important XML Concepts
- **Elements**: Named containers for data (e.g., `<link>`, `<joint>`)
- **Attributes**: Properties of elements (e.g., `name="left_arm"`)
- **Children**: Nested elements within parent elements
- **Self-closing tags**: Elements that don't contain children (e.g., `<origin xyz="0 0 0"/>`)

## URDF Elements and Structure

### Core Elements

A URDF file consists of the following core elements:

- `<robot>`: The top-level element containing the entire robot description
- `<link>`: Represents a rigid body (e.g., arm segment, torso)
- `<joint>`: Connects two links with a specified degree of freedom
- `<material>`: Defines appearance properties (color, texture)
- `<gazebo>`: Gazebo-specific simulation properties (plugins, physics)

### Link Element

A link describes a rigid body in the robot. It may contain:

- `<inertial>`: Mass, center of mass, and moment of inertia
- `<visual>`: Visual representation for rendering
- `<collision>`: Collision representation for physics simulation

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="1.0 1.0 1.0" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="1.0 1.0 1.0" />
    </geometry>
  </collision>
</link>
```

### Joint Element

A joint connects two links with a specified kinematic relationship:

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name" />
  <child link="child_link_name" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
</joint>
```

## Designing Humanoid Kinematics

### Human Anatomy Basics

Humanoid robots aim to replicate human-like anatomy and movement patterns. Key anatomical components include:

- **Torso**: Central body connecting limbs
- **Upper Arms**: Shoulders, elbows, wrists
- **Lower Arms**: Hands and fingers
- **Upper Legs**: Hips, knees
- **Lower Legs**: Ankles and feet
- **Head**: Neck connection, sensory devices

### Degrees of Freedom (DOF)

Humanoid robots typically have 20+ degrees of freedom to achieve human-like movement:

- **Torso**: 2-3 DOF (pitch, yaw, roll)
- **Legs**: 6 DOF each (hip: 3 DOF, knee: 1 DOF, ankle: 2 DOF)
- **Arms**: 7 DOF each (shoulder: 3 DOF, elbow: 1 DOF, wrist: 3 DOF)
- **Head/Neck**: 2-3 DOF (yaw, pitch, roll)

For our curriculum humanoid, we'll target a 28-DOF model for full functionality.

## Creating URDF for Humanoid Robots

### Basic Humanoid Skeleton

Here's a simplified URDF example for a humanoid robot with a basic skeleton:

```xml
<?xml version="1.0" ?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.15 0.1" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.15 0.1" />
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0.25" />
      <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.15 0.5" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.25" rpy="0 0 0" />
      <geometry>
        <box size="0.15 0.15 0.5" />
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0.1" rpy="0 0 0" />
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.05" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.5" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-0.785" upper="0.785" effort="50" velocity="2" />
  </joint>

  <!-- Left Arm (Simplified) -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.2" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.2" />
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="left_upper_arm" />
    <origin xyz="0.05 0.15 0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>

  <!-- Right Arm (Simplified) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.2" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.2" />
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="right_upper_arm" />
    <origin xyz="0.05 -0.15 0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>

  <!-- Left Leg -->
  <link name="left_thigh">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.04" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.4" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.4" />
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso" />
    <child link="left_thigh" />
    <origin xyz="-0.05 0.1 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="150" velocity="1" />
  </joint>

  <!-- Right Leg -->
  <link name="right_thigh">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.04" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.4" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.4" />
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="torso" />
    <child link="right_thigh" />
    <origin xyz="-0.05 -0.1 -0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="150" velocity="1" />
  </joint>
</robot>
```

## Physical Properties in URDF

### Inertial Properties

Accurate inertial properties are essential for realistic simulation. For each link, you must specify:

- Mass
- Center of mass location
- Moment of inertia tensor

The moment of inertia tensor for common shapes:
- Box: `ixx = 1/12 * m * (h² + d²)`
- Cylinder: `ixx = iyy = 1/12 * m * (3r² + h²)`, `izz = 1/2 * m * r²`
- Sphere: `ixx = iyy = izz = 2/5 * m * r²`

### Joint Limits

For humanoid robots, joint limits should reflect human range of motion:
- Shoulder: ±180° for abduction/adduction, ±90° for flexion/extension
- Elbow: 0° to 150° (flexion only)
- Hip: ±45° for abduction/adduction, -15° to 120° for flexion/extension
- Knee: 0° to 150° (flexion only)

## Visual Properties and Materials

### Geometry Shapes

URDF supports several basic geometric shapes:
- `<box>`: Rectangular cuboid
- `<cylinder>`: Cylinder with specified radius and length
- `<sphere>`: Sphere with specified radius
- `<mesh>`: Complex 3D models in OBJ, DAE, or STL format

### Defining Materials

Materials specify the visual appearance of links:

```xml
<material name="blue">
  <color rgba="0 0 1 1" />
</material>

<material name="green">
  <color rgba="0 1 0 1" />
</material>

<material name="red">
  <color rgba="1 0 0 1" />
</material>

<material name="white">
  <color rgba="1 1 1 1" />
</material>

<!-- You can also reference textures -->
<material name="human_skin">
  <color rgba="1 0.89 0.71 1" />  <!-- Beige/skin color -->
</material>
```

## Validation and Testing

### Validating URDF Files

Before using your URDF model, validate it with:

```bash
check_urdf /path/to/your/robot.urdf
```

This will check for:
- Proper XML syntax
- Consistent joint definitions
- Complete kinematic chains
- Valid inertial parameters

### Visualizing URDF

Use RViz to visualize your robot model:

```bash
ros2 run rviz2 rviz2
```

Then add a RobotModel display and set the robot description to your URDF file.

### Testing in Simulation

Load your model in Gazebo to test physics behavior:

```bash
# Launch your robot in Gazebo
ros2 launch gazebo_ros spawn_entity.py entity:=my_robot robot_namespace:=humanoid_robot -urdf -file /path/to/robot.urdf
```

## Integration with Simulation

### Gazebo Plugins

To make your humanoid robot functional in simulation, you'll need to add Gazebo plugins. Here's an example joint controller plugin:

```xml
<gazebo>
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <joint_name>left_shoulder_joint</joint_name>
    <joint_name>right_shoulder_joint</joint_name>
    <joint_name>left_hip_joint</joint_name>
    <joint_name>right_hip_joint</joint_name>
  </plugin>
</gazebo>
```

### Physics Properties

Specify physics properties for realistic simulation:

```xml
<gazebo reference="left_upper_arm">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Blue</material>
</gazebo>
```

## Advanced Topics

### Xacro Macros

For complex humanoid models, use Xacro to simplify URDF creation:

```xml
<?xml version="1.0" ?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define parameters once -->
  <xacro:property name="torso_height" value="0.5" />
  <xacro:property name="arm_length" value="0.6" />
  <xacro:property name="leg_length" value="0.8" />

  <!-- Define a macro for creating arms -->
  <xacro:macro name="humanoid_arm" params="side parent">
    <link name="${side}_upper_arm">
      <inertial>
        <mass value="2.0" />
        <origin xyz="0 0 0.1" />
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
      </inertial>
      <visual>
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <geometry>
          <cylinder radius="0.05" length="0.2" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <geometry>
          <cylinder radius="0.05" length="0.2" />
        </geometry>
      </collision>
    </link>
    
    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}" />
      <child link="${side}_upper_arm" />
      <origin xyz="0 ${sign(side)*0.15} 0.4" rpy="0 0 0" />
      <axis xyz="0 1 0" />
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:humanoid_arm side="left" parent="torso" />
  <xacro:humanoid_arm side="right" parent="torso" />
</robot>
```

### Sensor Integration

Humanoid robots typically have various sensors that need to be defined in URDF:

```xml
<!-- IMU sensor -->
<sensor name="imu_sensor" type="imu">
  <origin xyz="0 0 0.6" rpy="0 0 0" />
  <parent link="torso" />
  <always_on>true</always_on>
  <update_rate>50</update_rate>
</sensor>

<!-- Camera sensor -->
<sensor name="camera" type="camera">
  <origin xyz="0 0 0.1" rpy="0 0 0" />
  <parent link="head" />
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
</sensor>
```

## Exercises

### Exercise 1: Basic Humanoid Creation

Create a URDF file for a simplified humanoid robot with:
- Torso, head, 2 arms, 2 legs
- Use appropriate joint types and limits
- Add visual and collision elements
- Validate the URDF file

### Exercise 2: Adding Sensors

Extend your humanoid model with:
- An IMU in the torso
- A camera in the head
- Two LiDAR sensors on the sides of the head
- Verify sensor placement in RViz

### Exercise 3: Creating a Walking Animation

1. Create a ROS 2 node that publishes joint states to make the humanoid walk
2. Use the joint_state_publisher or create a custom node
3. Implement a simple walking gait pattern

## Summary

URDF is fundamental to humanoid robotics in ROS 2, providing a standardized way to describe robot models. For humanoid robots specifically:

- Kinematic structure must reflect human-like movement patterns
- Physical properties must be realistic for accurate simulation
- Materials and visuals enhance realism in simulation
- Proper validation ensures compatibility with simulators and controllers

## Knowledge Check

1. What are the essential elements of a URDF file?
2. How do you define the kinematic chain for a humanoid robot?
3. What are appropriate joint limits for humanoid shoulder joints?
4. How do you validate URDF files for correctness?
5. Why are inertial properties important in URDF?

## Resources

- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [Gazebo Model Tutorial](http://gazebosim.org/tutorials?tut=ros2_overview)
- [xacro Tutorial](http://wiki.ros.org/xacro)

---

<nav>
  <div class="prev-next-pagination" layout="row" align="center" justify="between">
    <a href="/docs/modules/03-python-to-ros-bridge-exercises"><- Previous: Python-to-ROS Bridge Exercises</a>
    <a href="/docs/modules/05-basic-ros2-examples">Next: Basic ROS 2 Examples -></a>
  </div>
</nav>