# 18-DOF High-Fidelity Vehicle Dynamics & Control Sim

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.8+-yellow)
![Status](https://img.shields.io/badge/Status-Active%20Development-green)
![Physics](https://img.shields.io/badge/Physics-18%20DOF-red)

## About The Project

Welcome to my vehicle dynamics sandbox! This project is a high-fidelity simulation environment built entirely on **ROS2**. 

The goal here isn't just to make a car move; it's to simulate the "messy" physics of real-world driving to test advanced control strategies. I'm simulating a full **18 Degrees of Freedom (DOF)** model with complex **2D LuGre tire friction**, which makes the handling significantly more realistic (and challenging) than standard bicycle models.

Currently, I'm using an **LQR controller** for torque vectoring, but I've hit a ceiling: the controller assumes it knows the friction perfectly, which it rarely does in real life, while the physics run the lugre model the controller, and the reference for it, run a pacejka aproximation. To fix this, I'm actively working on a **Physics-Informed Neural Network (PINN)** to estimate friction in real-time.

---

##System Architecture

The project is modularized into ROS2 nodes to keep the physics, control, and inputs estimating separate.

### Node Structure

* **`vehicle_dynamics_node`**: The physics engine. It calculates the 18 DOF rigid body dynamics and the 2D LuGre tire forces.
* **`controller_node`**: Runs the LQR optimization. It takes the state vector and calculates the optimal torque split for each wheel.
* **`steering_input_node`**: Handles user inputs (or trajectory following) and publishes steering commands.
* **`pinn_observer_node`** *(In Progress)*: The neural network observer that watches the dynamics and attempts to guess the friction coefficient ($\mu$) for the controller.

---

##Key Features

### 1. 18-DOF Physical Model
Most sims stop at 6 or 14 DOF. This model includes:
* **Chassis (6 DOF):** Surge, Sway, Heave, Roll, Pitch, Yaw.
* **Wheels (12 DOF):** Each corner has independent Vertical motion, Spin, and Steering compliance.

### 2.2D LuGre Friction
I ditched the standard Pacejka curves for the **LuGre model**. This simulates the microscopic "bristles" in the tire contact patch.
* **Importance:** It captures the transient behavior of friction—the tiny delay between applying force and generating grip, which is critical for high-performance control.

### 3. LQR Torque Vectoring
The vehicle uses a Linear Quadratic Regulator to stabilize the yaw rate and sideslip angle.
* *Current Limitation:* It relies on accurate state estimation. If the theres mismatch with the physics constants the controller is not effective.

### 4. PINN Friction Observer (In Development)
This is the active research part of the repo.I generated data using the model developed with varios maneuvers.
I'm training a recurrent Physics-Informed Neural Network to observe the vehicle's motion and "back-calculate" the friction coefficient.
It uses a GRU layer at the input with (V_slip,Fz)
* It combines data (vehicle states) with physics (LuGre equations) to estimate values.

---

## Getting Started

### Prerequisites
* Docker, I have set up the container for ease of installation and development

### Installation

1.  **Clone the repo:**
    ```bash
    mkdir -p vehicle_ws/src
    cd vehicle_ws/src
    git clone [tbh](tbd)
    ```

2.  **Build the workspace:**
    ```bash
    
    colcon build --packages-select tv_sim
    source install/setup.bash
    ```

3.  **Run the Simulation:**
    ```bash
    ros2 launch tv_sim launch.py
    ```

---

## To Do

* [x] **Core Physics:** Implement 18 DOF equations of motion.
* [x] **ROS2 Nodes:** Node development to run the simulation "Real Time".
* [x] **Tire Model:** Implement 2D LuGre friction logic.
* [x] **Control:** Basic LQR Torque Vectoring implementation.
* [x] **Data Generation:** Generated dataset for PINN training
* [ ] **Observer:** Finish development of neural based observer.
* [ ] **Optimization:** Tune LQR weights based on PINN friction feedback.
* [ ] **Viz:** Add Foxglove/Rviz visualizations for tire force vectors.

---

## 🤝 Contributing

If you're into vehicle dynamics or deep learning, feel free to use for simulating and devolping.

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.
