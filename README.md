# Persistent Monitoring System Simulation

## Description
A MATLAB-based simulation framework for persistent monitoring systems, where autonomous agents monitor multiple targets in a dynamic environment. The simulation demonstrates the interaction between mobile agents and stationary targets, with real-time visualization of system states and performance metrics.

## Key Features
- **Dynamic Target States**: Targets with time-varying uncertainty levels
- **Autonomous Agents**: Mobile agents with battery management and intelligent monitoring behavior
- **Real-time Visualization**: Interactive 2D display of system state, including:
  - Target uncertainty levels
  - Agent positions and battery status
  - System objective values
  - Power outage indicators
- **Energy-Aware Operation**: Battery management and power outage simulation
- **Performance Metrics**: Global objective tracking and system state monitoring

## Components

### Target.m
- Manages target state dynamics
- Implements uncertainty growth/reduction mechanisms
- Tracks individual and global objective values
- Supports different monitoring modes

### Agent.m
- Controls agent movement and behavior
- Implements battery management
- Handles different operational modes (traveling, dwelling, planning, power outage)
- Energy consumption modeling

### SimulationVisualizer.m
- Real-time 2D visualization
- Dynamic updates of system states
- Interactive display of metrics and status
- Color-coded state representation

### main.m
- Main simulation loop
- System initialization
- Parameter configuration
- Simulation control

## Installation
1. Clone the repository
2. Open MATLAB
3. Navigate to the project directory
4. Run `main.m` to start the simulation

## Usage
The simulation can be configured through parameters in `main.m`:
- Adjust number of targets and agents
- Modify simulation time and update resolution
- Configure initial positions and parameters
- Customize visualization settings

## Requirements
- MATLAB R2020b or newer
- MATLAB base toolbox
