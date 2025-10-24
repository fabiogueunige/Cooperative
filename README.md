# Cooperative Robotics - MATLAB & PyBullet Simulations

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020a+-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)](https://www.python.org/)
[![PyBullet](https://img.shields.io/badge/PyBullet-3.0.8-green.svg)](https://pybullet.org/)

**Authors:** Andrea Chiappe, Alberto Di Donna, Fabio Guelfi, Samuele Viola

This repository contains advanced robotics control implementations for cooperative manipulation tasks using **Task Priority Inverse Kinematics (TPIK)** algorithms. The project includes simulations for underwater vehicle-manipulator systems (UVMS), bimanual manipulation, and cooperative multi-robot systems.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Projects Description](#projects-description)
- [Franka Simulator](#franka-simulator)
- [Contributing](#contributing)
- [License](#license)

---

## 🎯 Overview

This repository implements state-of-the-art robotics control algorithms for:

- **Underwater Vehicle-Manipulator Systems (UVMS)**: Control of robotic arms mounted on underwater vehicles
- **Bimanual Manipulation**: Coordinated control of two robotic arms performing cooperative tasks
- **Cooperative Manipulation**: Multi-agent systems with shared manipulation goals
- **Path Following with Constraints**: Advanced trajectory tracking with joint limits and safety constraints

All implementations use **Task Priority Inverse Kinematics (TPIK)** with the **iCAT algorithm** to handle multiple prioritized tasks simultaneously.

---

## 📁 Project Structure

```
Cooperative/
├── Ass1_Exercise3/                    # UVMS Control (DexROV/Robust)
│   └── matlab_scripts/
│       ├── MainDexrov.m              # Main simulation script for DexROV
│       ├── MainRobust.m              # Main simulation script for Robust
│       ├── InitUVMS.m                # UVMS initialization
│       ├── ComputeJacobians.m        # Jacobian computation
│       ├── ComputeTaskReferences.m   # Task reference computation
│       ├── UpdateMissionPhase.m      # Mission state machine
│       └── simulation_scripts/       # Utility functions
│
├── Ass2_BimanualManipulation/        # Bimanual Robot Control
│   ├── matlab_scripts_bimanual/
│   │   ├── main.m                    # Main control loop
│   │   ├── InitRobot.m               # Robot initialization
│   │   ├── ComputeJacobians.m        # Jacobian matrices
│   │   ├── ComputeTaskReferences.m   # Task velocity references
│   │   └── simulation_scripts/       # Support functions
│   └── python_simulator/
│       └── franka_panda_simulation.py # PyBullet simulator
│
├── Ass3_Cooperative Manipulation/    # Cooperative Multi-Robot Control
│   ├── matlab_scripts_cooperation/
│   │   ├── main.m                    # Main cooperative control
│   │   ├── InitRobot.m               # Dual robot setup
│   │   └── simulation_scripts/       # Cooperation utilities
│   └── README.md
│
├── PathFollowerBM/                   # Bimanual Path Following with LOS
│   └── matlab_scripts_bimanual/
│       ├── main.m
│       ├── GetLosPoint.m             # Line-of-Sight algorithm
│       └── InitGoal.m                # Goal management
│
├── PathFollowerCoop/                 # Cooperative Path Following
│   └── matlab_scripts_cooperation/
│       ├── main.m
│       └── GetLosPoint.m
│
├── Simulator/                        # Generic Franka Panda Simulator
│   └── franka_simulator/
│       ├── franka_panda_simulation.py
│       ├── requirements.txt
│       └── panda_robot/
│
└── README.md                         # This file
```

---

## ✨ Features

### General Features
- ✅ **Task Priority Inverse Kinematics (TPIK)** using iCAT algorithm
- ✅ **Real-time 3D visualization** using PyBullet physics engine
- ✅ **Multi-phase mission management** with automatic transitions
- ✅ **Joint limit avoidance** using activation functions
- ✅ **Minimum altitude constraints** for safety
- ✅ **UDP communication** between MATLAB and Python simulator

### UVMS Specific (Exercise 1)
- Underwater vehicle positioning and control
- Vehicle-manipulator coordination
- Horizontal attitude control
- Manipulability optimization
- Tool positioning tasks

### Bimanual/Cooperative Specific (Exercises 2 & 3)
- **Rigid constraint maintenance** between two end-effectors
- **Cooperative object manipulation**
- **Dual-arm coordination** using consensus algorithms
- **Line-of-Sight (LOS) path following**
- **Feasible cooperative velocities** computation

---

## 🔧 Requirements

### MATLAB Requirements
- MATLAB R2020a or higher
- Robotics System Toolbox
- DSP System Toolbox (for UDP communication)

### Python Requirements
- Python 3.7+
- PyBullet 3.0.8
- NumPy
- See `requirements.txt` in simulator folders

---

## 🚀 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/fabiogueunige/Cooperative.git
cd Cooperative
```

### 2. Install Python Dependencies
```bash
cd Simulator/franka_simulator
pip install -r requirements.txt
```

Or for specific projects:
```bash
cd Ass2_BimanualManipulation/python_simulator
pip install -r requirements.txt
```

### 3. MATLAB Setup
- Open MATLAB
- Navigate to the desired project folder (e.g., `Ass2_BimanualManipulation/matlab_scripts_bimanual`)
- Add the `simulation_scripts` folder to your MATLAB path

---

## 💻 Usage

### Running a Simulation

#### For Bimanual/Cooperative Projects:

1. **Start the PyBullet Simulator** (Terminal/Command Prompt):
   ```bash
   cd Ass2_BimanualManipulation/python_simulator
   python franka_panda_simulation.py
   ```

2. **Run the MATLAB Control Script** (MATLAB):
   ```matlab
   cd matlab_scripts_bimanual
   main
   ```

#### For UVMS Project (Exercise 1):

1. **Start the Unity Simulator** (if available) or use MATLAB-only mode
2. **Run the MATLAB Script**:
   ```matlab
   cd Ass1_Exercise3/matlab_scripts
   MainDexrov  % or MainRobust
   ```

### Configuration

Edit the main MATLAB scripts to change:
- `real_robot = false` → Set to `true` for real robot control
- `end_time` → Simulation duration
- `deltat` → Integration time step
- Goal positions and mission phases

---

## 📚 Projects Description

### 🌊 Assignment 1: UVMS Control (Exercise 3)
**Objective:** Control an underwater vehicle-manipulator system to reach and manipulate objects.

**Key Tasks:**
- Tool positioning (T)
- Joint limits avoidance (JL)
- Minimum altitude maintenance (MA)
- Horizontal attitude control (HA)
- Manipulability optimization (MU)

**Files:** `Ass1_Exercise3/matlab_scripts/`

---

### 🤖 Assignment 2: Bimanual Manipulation
**Objective:** Two Franka Panda robots cooperatively manipulate a shared object.

**Mission Phases:**
1. **Go-to Phase**: Each arm moves to its grasping position
2. **Cooperative Manipulation**: Rigid constraint active, object manipulation
3. **End Motion**: Controlled stop

**Key Tasks:**
- Tool positioning (T)
- Rigid constraint (RC) - maintains fixed relative pose between end-effectors
- Joint limits (JL)
- Minimum altitude (MA)

**Files:** `Ass2_BimanualManipulation/`

---

### 🤝 Assignment 3: Cooperative Manipulation
**Objective:** Multi-agent cooperative control with consensus-based algorithms.

**Algorithm Steps:**
1. Each agent computes non-cooperative task velocities
2. Agents exchange information
3. Cooperative velocity consensus via weighted average
4. Cartesian constraint matrix evaluation
5. Feasible cooperative velocities computation
6. Task-priority inverse kinematics with cooperative tasks

**Files:** `Ass3_Cooperative Manipulation/`

---

### 🎯 Path Follower Projects
**Objective:** Implement Line-of-Sight (LOS) algorithms for path following while respecting constraints.

**Features:**
- Dynamic goal generation along a path
- Joint limit avoidance
- Minimum altitude maintenance
- Smooth trajectory execution

**Files:** `PathFollowerBM/` and `PathFollowerCoop/`

---

## 🔗 Franka Simulator

The PyBullet-based Franka Panda simulator used in this project is available at:

**Repository:** [https://github.com/elchape99/franka_sim.git](https://github.com/elchape99/franka_sim.git)

The simulator provides:
- Real-time 3D visualization
- Physics-based simulation
- UDP communication with MATLAB
- Dual-robot setup support

---

## 🤝 Contributing

This is an academic project. For questions or suggestions:
- Open an issue on GitHub
- Contact the authors

---

## 📄 License

This project is developed for academic purposes. Please contact the authors for usage permissions.

---

## 📞 Contact

For questions or collaborations, please reach out to the development team:
- Andrea Chiappe
- Alberto Di Donna
- Fabio Guelfi
- Samuele Viola

---

## 🙏 Acknowledgments

- University of Genoa - Robotics Course
- Franka Emika for the Panda robot model
- PyBullet physics engine
- MATLAB Robotics Toolbox

---

**⭐ If you find this repository useful, please consider giving it a star!**
