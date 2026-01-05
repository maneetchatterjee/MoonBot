# Agile Rover–Manipulator Reinforcement Learning Framework a.k.a MoonBot

Simulation-first development of an autonomous rover with an articulated robotic arm using physics-based reinforcement learning.

This repository presents a **complete end-to-end robotics learning pipeline**, starting from **CAD-level mechanical design** and culminating in **robust locomotion policies trained via reinforcement learning** in high-fidelity physics simulators. The framework is **simulator-agnostic**, **scalable**, and **sim-to-real ready**.

---

## 🚀 Motivation

Modern mobile manipulators must operate under:

- Uncertain terrain conditions  
- Strict stability constraints  
- Energy-efficient actuation  
- Modeling inaccuracies and noise  

Rather than iterating directly on hardware, this project adopts a **simulation-driven learning approach**, enabling:

- Safe large-scale exploration  
- Rapid policy iteration  
- Domain randomization  
- Transferable control policies  

---

## 🧠 Key Contributions

- Full **CAD → URDF/Xacro → Physics Simulation** pipeline  
- **Gymnasium-compatible reinforcement learning environments**  
- **PPO-based locomotion training** with stability-aware reward shaping  
- **Domain randomization** for sim-to-real robustness  
- **Hierarchical RL–ready architecture** (locomotion + manipulation)  
- Cross-simulator compatibility: **PyBullet, MuJoCo, Isaac Sim**

---

## 🦾 Robot Overview

- **Platform**: 6-wheel rover with articulated suspension  
- **Manipulator**: Multi-DOF robotic arm mounted on rover base  
- **Control**:
  - Locomotion: Wheel velocity control  
  - Manipulation: Joint-space / task-space control (extensible)  
- **Simulated Sensors**:
  - IMU (base orientation, angular velocity)  
  - Wheel encoders  
  - Base linear velocity  
  - Extendable to RGB-D / LiDAR  

---

## 📁 Repository Structure

```text
agile-rover-rl/
├── assets/                  # Visual & collision meshes
│   ├── meshes/
│   │   ├── visual/
│   │   └── collision/
│
├── urdf/                    # Robot description files
│   ├── rover_base.xacro
│   ├── rover_arm.xacro
│   └── rover.urdf
│
├── envs/                    # Gymnasium environments
│   ├── rover_locomotion_env.py
│   ├── rover_arm_env.py
│   └── reward_functions.py
│
├── rl/                      # Reinforcement learning utilities
│   ├── domain_randomization.py
│   ├── callbacks.py
│   └── policies.py
│
├── train/                   # Training scripts
│   ├── train_locomotion_ppo.py
│   ├── train_hierarchical.py
│   └── hyperparams.yaml
│
├── eval/                    # Evaluation & metrics
│   ├── evaluate_policy.py
│   └── metrics.py
│
├── utils/                   # Supporting utilities
│   ├── mesh_utils.py
│   └── urdf_utils.py
│
├── results/                 # Logs, checkpoints, plots
│
├── requirements.txt
└── README.md
````

---

## 🧱 CAD → Simulation Pipeline

### Mechanical Design

* Full rover and articulated manipulator designed in **SolidWorks**
* Mechanical constraints (joint limits, mounting offsets, clearances) defined at CAD level

### Mesh Processing

* **Visual meshes**: High-resolution (`.DAE` / `.OBJ`)
* **Collision meshes**: Simplified (`.STL`)
* Unit conversion from **millimeters to meters**
* Origin alignment at joint frames
* Mesh decimation to reduce triangle count and improve simulation stability

### URDF / Xacro Modeling

* Accurate **inertial tensors** derived from CAD geometry
* Explicit **joint limits, effort, and velocity constraints**
* Modular robot description separating **rover base** and **manipulator arm**
* Xacro-based parameterization for easy scaling and reuse

### Physics Simulation

* **PyBullet** for rapid prototyping and reinforcement learning iteration
* **MuJoCo** for high-fidelity contact dynamics and smooth control
* **NVIDIA Isaac Sim** for photorealistic simulation and sim-to-real transfer

---

## 🤖 Reinforcement Learning Formulation

### Observation Space

* Base linear velocity (3 DOF)
* Base angular velocity (3 DOF)
* Base orientation (roll, pitch, yaw)
* Wheel joint positions

### Action Space

* Continuous wheel velocity commands (4 DOF)

### Reward Function (Locomotion)

The reward function balances:

* Forward velocity maximization
* Energy efficiency
* Roll / pitch stability
* Suppression of lateral and vertical motion

```math
R = 2.0·v_x − 2.5·(|roll| + |pitch|)
    − 0.001·||a||² − 0.3·|v_y| − 0.5·|v_z|
```

---

## 🧪 Training

### Locomotion Training (PPO)

```bash
python train/train_locomotion_ppo.py
```

* Algorithm: **Proximal Policy Optimization (PPO)**
* Policy: Multi-layer perceptron (MLP)
* Training horizon: **600k+ timesteps**
* Physics parameters randomized at every episode reset

---

## 🔁 Domain Randomization

To improve sim-to-real transfer, the following parameters are randomized:

* Link mass randomization (±15%)
* Friction coefficient randomization
* Motor strength scaling
* Actuation noise injection

---

## 🧠 Extensibility

The framework is designed to support:

* Hierarchical reinforcement learning (locomotion + manipulation)
* Vision-based end-to-end policies (RGB / depth)
* Curriculum learning strategies
* Multi-objective reward optimization
* Multi-agent extensions
* Real-world robotic deployment

---

## 🌍 Sim-to-Real Outlook

The architecture facilitates reliable sim-to-real transfer through:

* Conservative collision geometry modeling
* Latency-aware actuation assumptions
* Parameter identification compatibility
* Safe deployment pathways for physical hardware

---

## 🛠 Tools & Technologies

* **CAD & Modeling**: SolidWorks, Blender, MeshLab
* **Simulation**: PyBullet, MuJoCo, NVIDIA Isaac Sim
* **Learning**: Stable-Baselines3 (PPO), Gymnasium
* **Programming**: Python
* **Logging & Analysis**: NumPy, TensorBoard

---

## 📌 Future Work

* Full MuJoCo MJCF implementation
* Hierarchical reinforcement learning (locomotion + manipulation)
* Vision-based perception-driven policies
* Isaac Sim photorealistic training
* Real-robot validation and benchmarking

---

## 📄 License

MIT License

---

## 📬 Contact

For collaboration, research extensions, or academic discussion, feel free to reach out.
