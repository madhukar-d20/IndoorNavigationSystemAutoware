# Indoor Navigation System for Car Parks
### Autoware-Based GNSS-Denied Autonomous Navigation & Digital Twin

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Autoware](https://img.shields.io/badge/Framework-Autoware-orange)
![Python](https://img.shields.io/badge/Language-Python-lightblue)
![Platform](https://img.shields.io/badge/Platform-Linux%20Ubuntu-lightgrey)
![Status](https://img.shields.io/badge/Status-In%20Progress-yellow)

---

## Overview

This repository contains the implementation of an **indoor vehicle navigation prototype** and **cloud-connected Digital Twin** for GNSS-denied environments such as underground car parks. The system is developed as part of a **Master Research Project Thesis** at Fachhochschule Dortmund (FH Dortmund), supervised by **Prof. Dr.-Ing. Björn Schäfer** (Smart Mobility).

The core challenge: standard navigation systems rely on GPS/GNSS signals which are unavailable inside buildings. This project addresses that by building a full autonomous navigation stack using **static point cloud maps**, **semantic Lanelet2 routing**, and **cloud-to-robot digital twin integration** — all verified through a Software-in-the-Loop (SITL) architecture using the **Autoware Planning Simulator**. 

The system's geometry and routing logic are strictly modeled and tested over the structural baseline of the **FH Dortmund (Sonnenstraße campus) parking garage**.

---

## Problem Statement

Conventional navigation systems guide vehicles based on GNSS and static digital maps. Inside a parking garage:

- **No GNSS signal** is available.
- Parking space availability changes dynamically, requiring real-time state synchronization.
- There is a significant architectural gap between high-level cloud booking applications and the low-level kinematic trajectory planners running on the robot.

This project solves these challenges by deploying a decoupled architecture: leveraging a FIWARE context broker for data persistence, a FastAPI/WebSocket network for agile goal dispatching, and the Autoware ecosystem for precise lane-level routing and kinematic control.

---

## Key Features

- **Software-in-the-Loop (SITL) Validation** utilizing the Autoware Planning Simulator for safe, reliable, and repeatable testing of autonomous routing logic.
- **FIWARE NGSI-LD Context Broker** integration to serve as the authoritative Digital Twin, tracking real-time spot occupancy and infrastructure status.
- **FastAPI Mission Control Server** featuring an automated, preference-aware spot selection algorithm (prioritizing EV and accessible spaces).
- **Asynchronous WebSocket Bridge** that intercepts cloud-based booking payloads and pushes coordinates to the local ROS 2 stack with <500ms latency.
- **Dual-Interface HMI** combining a local Python Tkinter desktop application and a Flask web dashboard to display proximity-triggered navigation instructions.
- **Semantic Lanelet2 Vector Map** of the FH Dortmund Sonnenstraße facility, meticulously constructed via Vector Map Builder to include priority routing rules, structural clearances, and 10 distinct parking nodes.

---

## Tech Stack

| Component | Technology |
|---|---|
| **Autonomous Framework** | Autoware (Universe) |
| **Middleware** | ROS 2 (Humble Hawksbill) |
| **Cloud & API Backend** | FastAPI, WebSockets, Cloudflare Tunnels |
| **Digital Twin / IoT** | FIWARE Orion Context Broker (NGSI-LD) |
| **HMI & Dashboards** | Flask, Python Tkinter |
| **Map Format** | Lanelet2, PCD (Point Cloud Data) |
| **Map Building Tool** | Vector Map Builder |
| **Primary Language** | Python 3.10+ |
| **OS** | Ubuntu 22.04 LTS |
| **Target Hardware (Future)** | Innok Heros Robot, Yuhesen FR Max Robot |

---

## Repository Structure

```text
IndoorNavigationSystemAutoware/
│
├── autoware/
│   ├── map/               # Map assets for FH Dortmund Sonnenstraße campus
│   │                      # (Contains the .osm semantic vector map)
│   │
│   └── src/               # Autoware package customizations
│                          # and indoor navigation configuration files
│
├── ros2_ws/
│   └── src/               # Custom ROS 2 nodes
│      └── parking_system/
|        └── parking_system/
|           |── integrated_dashboard_node.py  # ROS 2 tracking node + Flask HMI
|           |── websocket_bridge.py           # Async bridge translating WSS events to ROS 2 HTTP POSTs
│           └── advanced_tkinter_gui.py       # GUI developed for parking simulation during the first phase of the project
|
│
└── .gitignore
```

---

## Environment Setup

| Component | Specification |
|---|---|
| **Simulation Mode** | Autoware Planning Simulator (Software-in-the-Loop) |
| **Test Environment Map** | Car park, Fachhochschule Dortmund, Sonnenstraße campus, Dortmund |

---

## Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- Autoware Universe ([installation guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/))
- Python Packages: `fastapi`, `uvicorn`, `websockets`, `requests`, `flask`
- Docker (for running the FIWARE Orion Context Broker)

### Installation

```bash
# Clone the repository
git clone https://github.com/madhukar-d20/IndoorNavigationSystemAutoware.git
cd IndoorNavigationSystemAutoware

# Build ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running the System (SITL Pipeline)

**1. Launch Autoware Planning Simulator:**
```bash
# Launch Autoware with the Sonnenstraße indoor map configuration
ros2 launch autoware_launch planning_simulator.launch.xml
  map_path:=<path_to_sonnenstrasse_map>
  vehicle_model:=sample_vehicle
  sensor_model:=sample_sensor_kit
```

**2. Start the Cloud Backend & Digital Twin:**
```bash
# Ensure FIWARE Orion is running via Docker, then start FastAPI

```

**3. Start the ROS 2 HMI Node & WebSocket Bridge:**
```bash
# Start the local Flask/Tkinter Node
ros2 run <your_custom_package> integrated_dashboard_node

# In a new terminal, start the bridge to listen for cloud tasks
python3 websocket_bridge.py
```

---

## Current Status

This project is actively under development as part of an ongoing Master's thesis (March 2025 – September 2025).

- [x] Literature review on Autoware for GNSS-denied environments
- [x] Point cloud map processing for FH Dortmund Sonnenstraße campus
- [x] Semantic Lanelet2 Map construction (10 Parking Spots)
- [x] ROS 2 workspace and Autoware planning simulator integration
- [x] Dual-Interface HMI implementation (Flask + Tkinter)
- [x] Digital twin integration (FIWARE Orion Context Broker)
- [x] Cloud-to-Robot WebSocket coordination pipeline
- [x] Software-in-the-Loop (SITL) path planning validation
- [ ] End-to-end system testing on physical robot platform (Future Work)
- [ ] Thesis submission

---

## Academic Context

| | |
|---|---|
| **Institution** | Fachhochschule Dortmund |
| **Program** | M.Eng. Embedded Systems Engineering |
| **Module** | Research Project (Thesis) — MOD3-03 |
| **Supervisor** | Prof. Dr.-Ing. Björn Schäfer, Smart Mobility |
| **Test Location** | Fachhochschule Dortmund, Sonnenstraße campus, Dortmund |

---

## References

- [Autoware Foundation](https://autoware.org/autoware-overview/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Lanelet2 Map Format](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [Vector Map Builder](https://tools.tier4.jp/)
- [FIWARE NGSI-LD Specification](https://www.fiware.org/developers/smart-data-models/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)

---

## Author

**Madhukar Devendrappa** M.Eng. Embedded Systems Engineering, FH Dortmund  
[GitHub](https://github.com/madhukar-d20) · [LinkedIn](https://www.linkedin.com/in/madhukar-devendrappa-135ba7171/)

---

*This repository is part of an active academic research project. Code and documentation are continuously updated.*
