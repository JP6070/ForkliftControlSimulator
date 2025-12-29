# Forklift Control Simulator
## Overview

This project is a console-based forklift lift control simulator written in C++. It models the vertical lift mechanism of a forklift using a PLC-style scan loop and state machine, with a strong focus on safety, predictability, and fault handling rather than UI or graphics.

The simulator runs in real time with a fixed update rate and allows manual interaction via keyboard commands to simulate operator inputs such as lifting, lowering, emergency stops, and fault resets.

The design intentionally mirrors industrial control logic rather than typical application-style programming.

## Key Design Goals

* Model realistic lift behaviour using a simple physical plant
* Enforce safe state transitions
* Handle faults deterministically using priority-based latching
* Separate inputs, control logic, plant dynamics, and outputs

## The System Is Split Into Four Clear Layers

### 1. Inputs (Sensors & Operator Commands)
The Inputs struct represents everything the controller can see in a single scan cycle: 

* Operator commands (cmdUp, cmdDown, cmdHold)
* Safety signals (estop)
* Limit switches (topLimit, bottomLimit)
* Load measurement (loadKg)
* Fault reset pulse (resetFault)

### 2. Controller (Decision Logic)

The LiftController is the core of the system and acts like a PLC program:

* Runs once per scan
* Latches faults before making decisions
* Chooses a single LiftState (Holding, Lifting, Lowering, Faulted)
* Produces outputs based only on the current state and inputs
* Fault handling is priority-based, meaning higher-severity faults override lower ones and remain latched until safely reset.

This prevents unsafe behaviour such as:

* Driving into a limit switch
* Restarting while still moving
* Clearing faults while emergency stop is active

### 3. Plant Model (Physical Behaviour)

The LiftPlant simulates the physical lift mechanism:

* Position ranges from 0.0 (bottom) to 1.0 (top)
* Velocity changes gradually to simulate inertia
* Movement is clamped to physical limits

The controller never sets position directly. It only commands a target velocity, which the plant follows.

### 4. Outputs (Actuators & Indicators)

The Outputs struct models what the controller can control:

* Motor enable
* Motor direction
* Brake engagement
* Fault indicator lamp

This ensures the controller logic stays independent of how outputs are eventually applied.

## Fault Handling Philosophy

Faults are:

* Latched (they persist until explicitly cleared)
* Prioritised (emergency stop overrides overload, etc.)
* State-forcing (any fault immediately moves the system to Faulted)

A fault reset is only allowed when:

* Emergency stop is released
* The lift is effectively stationary

## Console Simulation

The main() function provides a simple interactive console to drive the system:

### Commands

u = lift up <br>
d = lower down <br>
h = hold <br>
s = stop all commands <br>
e = toggle emergency stop <br>
r = reset fault (only when safe) <br>
l = set load weight <br>
q = quit <br>

The simulation runs at a fixed 20 ms update rate, similar to a real PLC scan time, and prints system state at regular intervals.
