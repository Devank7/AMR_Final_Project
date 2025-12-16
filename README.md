# AMR Final Project: 2-Wheel Differential Drive Robot

## Project Summary

This project presents the complete **modeling, control, and motion planning** of a **two‑wheeled differential drive mobile robot** equipped with **two actuated drive wheels**. The objective is to demonstrate how kinematic and dynamic models, feedback control, and motion planning algorithms can be integrated to achieve **stable trajectory tracking and reliable obstacle avoidance** in a planar environment.

The system is implemented and validated through Python-based simulations and animations.

---

## 1. System Modeling

### Kinematic Modeling

* Derived the **forward and inverse kinematics** of a differential drive robot
* Related individual wheel velocities to the robot’s linear and angular motion
* Modeled the robot as a **non‑holonomic system** operating in 2D space

### Dynamic Modeling

* Implemented simplified **wheel and motor dynamics**
* Included actuator dynamics to capture realistic velocity tracking behavior
* Integrated the dynamic model with kinematics for closed-loop simulation

### Physical Principles and Implementation

* Independent wheel actuation generates both translation and rotation
* A passive caster wheel provides mechanical stability without influencing kinematics
* All models are implemented in **Python** and validated via simulation

---

## 2. Control Methodology

### Control Strategy

* Designed **SMC Controllers** for individual wheel velocity control
* Used inverse kinematics to map desired linear and angular velocities to wheel commands
* Achieved smooth motion for both straight-line and curved trajectories

### Stability Considerations

* Analyzed closed-loop behavior using velocity error dynamics
* Demonstrated **stability** under SMC
* Ensured bounded tracking errors under realistic actuator limits

### Testing and Validation

* **Straight-line motion:** constant linear velocity with zero angular velocity
* **Circular motion:** constant linear and angular velocities
* Performance validated using trajectory tracking plots and animations

---

## 3. Motion Planning and Obstacle Avoidance

### Planning Approach

* Implemented **artificial potential field–based motion planning**
* Combined attractive forces toward the goal with repulsive forces from obstacles, pedestrians, walls
* Able to predict the possible path/trajectory of the robot from start to goal adn then improvise

### Obstacle Avoidance

* Modeled obstacles using inflated safety (no‑go) zones
* Enabled collision-free navigation around multiple moving or static obstacles
* Ensured compatibility with differential drive non‑holonomic constraints

### Key Characteristics

* Reactive, real-time planning
* Smooth and continuous trajectories
* Seamless integration with low-level control

---

## Results

* Successful navigation from start to goal locations
* Stable straight-line and circular trajectory tracking
* Reliable obstacle avoidance in environments with multiple moving/static obstacles
* Animations confirm smooth motion, stable control, and goal convergence

---

## Repository Structure

* **AMR_Project_Presentation_Devank.pptx**
  Final presentation summarizing modeling, control, motion planning, results, and challenges with solutions.


All the files named AMR_HW... have simulations for:
Straight-line motion simulation of a 2-wheeled differential drive robot
(linear velocity v = x, angular velocity ω = 0).

Circular motion simulation of a 2-wheeled differential drive robot
(linear velocity v = x, angular velocity ω = y).

Motion planning and obstacle avoidance simulation using Artificial Potential Fields, Control Barrier Function.
Demonstrates navigation around various obstacles static/moving with safety zones and animation.

---

## Setup and Installation

```bash
pip install -r requirements.txt
```

Once dependencies are installed, each script can be executed independently to reproduce the corresponding simulations and animations.
