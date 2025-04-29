# Balloon-Assisted Drone Flight Simulator
This repository contains a flight simulator developed as part of a master’s thesis project in the Aerospace Engineering Department at Iowa State University. Designed for use in academic courses, the simulator provides students with a hands-on tool to explore and analyze aircraft flight dynamics, particularly those of a balloon-assisted drone.

## Table of Contents
- [Abstract](#abstract)
- [Demonstrations](#demonstrations)
- [Required Files](#required-files)
- [Installation](#installation)
- [Configuration](#configuration)
- [How to Run](#how-to-run)


## Abstract
In the field of Aerospace Engineering, the development of flight simulators has given engineers that design aircraft as well as pilots the opportunity to learn more about an aircraft's flight performance prior to construction. Flight simulation serves as an essential tool for verifying aircraft dynamics and stability, ensuring that systems such as fly-by-wire circuits perform as intended under real-world conditions. This study presents a flight simulator to evaluate the flight dynamics of an aircraft throughout the design process. Specifically, the simulator models an indoor balloon-assisted drone aircraft to provide aerospace engineering students with the opportunity to evaluate flight characteristics for course projects. However, the core principles and simulation methods of the flight simulator are adaptable, making the tool suitable for a range of aircraft types and applications. The flight simulation represents how the aircraft flies through time—changing position, attitude, velocity, and acceleration. In this study, a dynamic model of an aircraft was developed to be the core of the flight simulator.  It uses fundamental principles of physics to determine the forces and moments acting on the aircraft. In the construction of this flight simulator, object-oriented programming techniques were used to model the aircraft and its various components, such as the craft, thrusters, and connection points. The simulation runs iteratively, generating real-time trajectories and flight paths that reflect the aircraft's performance. This approach provides a valuable resource for engineers and students to optimize aircraft systems and enhance flight safety prior to physical implementation.

## Demonstrations
Visual demonstrations, including example flight videos, screenshots of the user interface, and sample simulation results, are available in the [`demonstrations`](https://github.com/your-username/your-repo-name/tree/main/demonstrations) folder of the repository.

## Required Files
- quatpy.py from spatialnde2
- spatialnde2 (for flight visualization) [spatialnde2 GitHub](https://github.com/isuthermography/spatialnde2)
- dataguzzler-python (for running the simulation) [dataguzzler-python GitHub](https://github.com/isuthermography/dataguzzler-python)

## Installation 
1. Clone the repository:
   ```bash
   git clone https://github.com/ashleybehrendt/LTASimulator.git
   cd LTASimulator
2. Install Dependencies
    Install/build *spatialnde2* through above link (instructions provided in README.md)
    Install/build *dataguzzler-python through above link (instructions provided in README.md)

## Configuration 
   In LTASimulator, the LTA_config.py file is responsible for all aircraft specific configurations. Edit this file by adding in your aircraft specific configuration data.

## How to Run: 
1. To run the file via terminal:
    ```bash
    dgpy LTA_Run.dgp
2. The SNDE viewer will automatically appear upon startup. If you would like to view the 3D plots of the aircraft's motion, type the following into the terminal after desired run time.
    ```bash
   plot_history_now.append(True)
