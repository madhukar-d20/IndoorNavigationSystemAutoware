# Indoor Navigation System for Car Parks
### Autoware-Based GNSS-Denied Autonomous Navigation

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Autoware](https://img.shields.io/badge/Framework-Autoware-orange)
![Python](https://img.shields.io/badge/Language-Python-lightblue)
![Platform](https://img.shields.io/badge/Platform-Linux%20Ubuntu-lightgrey)
![Status](https://img.shields.io/badge/Status-In%20Progress-yellow)

---

## Overview

This repository contains the implementation of an **indoor vehicle navigation prototype** for GNSS-denied environments such as underground car parks. The system is developed as part of a **Master Research Project Thesis** at Fachhochschule Dortmund (FH Dortmund), supervised by **Prof. Dr.-Ing. Björn Schäfer** (Smart Mobility).

The core challenge: standard navigation systems rely on GPS/GNSS signals which are unavailable inside buildings. This project addresses that by building a full autonomous navigation stack using **LiDAR-based localization**, **point cloud maps**, and **digital twin integration** — all within the **Autoware autonomous driving framework**.

The system is deployed and tested on real robot platforms at the **car park of Klinikum Dortmund, Dortmund, Germany**.

---

## Problem Statement

Conventional navigation systems guide vehicles based on GNSS and static digital maps. Inside a parking garage:

- **No GNSS signal** is available
- Parking space availability changes dynamically
- Navigation must be precise enough to guide a vehicle to a specific bay

This project solves these challenges through sensor-based localization, digital twin data integration, and path planning within the Autoware ecosystem.

---

---

## Key Features

- **LiDAR-based localization** using NDT (Normal Distributions Transform) matching against a pre-built point cloud map — replacing GNSS in enclosed spaces
- **Digital twin integration** combining static parking space geometry with live occupancy and reservation data for dynamic navigation targets
- **Full Autoware navigation stack**: localization → map management → path planning → vehicle control
- **ROS 2 architecture** on Ubuntu Linux for modular, scalable node communication
- **Real-world deployment** on Innok Heros or Yuhesen FR Max robot platforms

---

## Tech Stack

| Component | Technology |
|---|---|
| Autonomous Driving Framework | Autoware (Universe) |
| Middleware | ROS 2 (Humble) |
| Localization | NDT Matching, Point Cloud Maps |
| Map Format | Lanelet2, PCD (Point Cloud Data) |
| Map Building Tool | Vector Map Builder |
| Primary Language | Python |
| Build System | CMake (colcon) |
| OS | Ubuntu 22.04 |
| Hardware | Innok Heros Robot, Yuhesen FR Max Robot |

---

## Repository Structure

```
IndoorNavigationSystemAutoware/
│
├── autoware/
│   └── src/               # Autoware package customizations
│                          # and configuration files
│
├── ros2_ws/
│   └── src/               # Custom ROS 2 nodes and packages
│                          # for sensor integration and control
│
├── src/                   # Core Python modules
│                          # (localization, digital twin interface,
│                          #  map management utilities)
│
└── .gitignore
```

---

## Hardware Setup

| Component | Specification |
|---|---|
| Robot Platform | Innok Heros / Yuhesen FR Max |
| Test Environment | Car park, Klinikum Dortmund, Hohe Straße 33, Dortmund |

---

## Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- Autoware Universe ([installation guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/))
- Python 3.10+
- colcon build tool

### Installation

```bash
# Clone the repository
git clone https://github.com/madhukar-d20/IndoorNavigationSystemAutoware.git
cd IndoorNavigationSystemAutoware

# Build ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Build Autoware workspace
cd ../autoware
colcon build --symlink-install
source install/setup.bash
```

### Running the Navigation Stack

```bash
# Launch Autoware with indoor navigation configuration
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=<path_to_pointcloud_map> \
  vehicle_model:=<robot_model> \
  sensor_model:=<lidar_model>
```

> **Note:** Full launch instructions and configuration files will be updated as the project progresses toward the submission deadline.

---

## Current Status

This project is actively under development as part of an ongoing Master's thesis (March 2025 – September 2025).

- [x] Literature review on Autoware for GNSS-denied environments
- [x] Concept definition for indoor car park navigation
- [x] Point cloud map acquisition at Klinikum Dortmund
- [x] LiDAR localization pipeline setup
- [x] ROS 2 workspace and Autoware integration
- [x] Digital twin interface implementation
- [x] Full path planning pipeline validation
- [ ] End-to-end system testing on robot platform
- [ ] Thesis submission

---

## Academic Context

| | |
|---|---|
| **Institution** | Fachhochschule Dortmund (FH Dortmund) |
| **Program** | M.Eng. Embedded Systems Engineering |
| **Module** | Research Project (Thesis) — MOD3-03 |
| **Supervisor** | Prof. Dr.-Ing. Björn Schäfer, Smart Mobility |
| **Test Location** | Klinikum Dortmund, Hohe Straße 33, 44139 Dortmund |
---

## References

- [Autoware Foundation](https://autoware.org/autoware-overview/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Lanelet2 Map Format](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [Vector Map Builder](https://tools.tier4.jp/)
---

## Author

**Madhukar Devendrappa**
M.Eng. Embedded Systems Engineering, FH Dortmund
[GitHub](https://github.com/madhukar-d20) · [LinkedIn](https://www.linkedin.com/in/madhukar-devendrappa-135ba7171/)

---

*This repository is part of an active academic research project. Code and documentation are continuously updated.*
