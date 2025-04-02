# Explicit-ArmBot

**Explicit-ArmBot** is a MATLAB-based framework for simulating, animating, and analyzing human arm movements and robot kinematics using an anatomically structured arm model. It supports subject-specific modeling from anthropometric data and includes animations and dynamic computations via Exponential Maps.

---

## Project Structure

```text
Explicit-ArmBot/
│
├── subject_data.xlsx        # Subject-specific anthropometric and demographic data
├── README.md                # This file
├── LICENSE                  # Licensing information
│
└── Explicit-MATLAB/
    ├── setup.m                 # Initialization script (MUST be run before simulations)
    ├── func_addSubfolders.m   # Adds all necessary subfolders to the MATLAB path
    │
    ├── examples/              # Main scripts for simulation (e.g., main_arm_sim.m)
    ├── animation/             # Real-time 3D animation utilities
    ├── robots/                # ArmBot and other robots, extending RobotPrimitive
    ├── graphics/              # VFC data for visualization
    ├── helpers_geometry/      # Geometry-related helper functions
    ├── interpolation/         # Symplectic Euler integrators
    └── utils/                 # Utility functions (e.g., video saving)

---

## Getting Started

1. **Clone the repository** or download it:
    ```bash
    git clone https://github.com/YOUR_USERNAME/Explicit-ArmBot.git
    ```

2. **Start MATLAB**, navigate to the `Explicit-MATLAB` directory, and run:

    ```matlab
    setup
    ```

    This sets up all required paths and dependencies.

3. **Simulate an arm**:

    Open and run the main script:
    ```matlab
    examples/main_arm_sim.m
    ```

---

## Subject-Specific Arm Simulation

Anthropometric and demographic data for each subject is stored in [`subject_data.xlsx`](../subject_data.xlsx). Columns include:

- `SubjectID` (e.g., S01, S02)
- `Sex` (`M` or `F`) – used for sex-specific biomechanical estimates
- `Age`
- `Handedness`
- `Height_m`, `Weight_kg`
- `UpperArm_m`, `Forearm_m`, `Hand_m` – segment lengths (shoulder→elbow, elbow→wrist, wrist→knuckles)

Each subject's arm model is generated based on Winter’s anthropometric data using their segment lengths and weight. The inertia and mass properties are automatically computed per segment.

You can select a specific subject by ID or loop through all subjects.

---

## Features

- Anatomically accurate 7-DOF arm (`ArmBot`) with shoulder, elbow, and wrist joints
- Subject-specific inertial properties (mass + scalar inertia)
- Real-time animation
- Modular architecture: swap in robots, animations, integrators, etc.
- Symplectic integration and energy-aware simulations
- Expandable to robot simulations and rehabilitation modeling

---

## Adding New Subjects

To simulate new participants:
1. Open `subject_data.xlsx`
2. Add a new row with the subject's info (fill in all required columns)
3. Re-run `main_arm_sim.m` or modify it to select your new subject.

---

## Saving Videos

To record animations as videos, utility functions in `/utils/` can be enabled. See `saveVideo_*` functions.

---

## 👨‍🔬 Authors

Johannes Lachner  
Postdoctoral Researcher, MIT  
Contact: jlachner[at]mit.edu

Alexander Dietrich
Head of the whole-body control group, Institute of Robotics and Mechatronics, DLR
Lecturer at TUM
Contact: Alexander.Dietrich[at]dlr.de

---

## 📄 License

This project is licensed under the terms of the [MIT License](LICENSE).
