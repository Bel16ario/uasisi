# UASISI - Modular wing simulation framework

UASISI is an extensible signal based simulation framework with features to facilitate modelling of problems involving fixed wings through the use of modules. Module selection is limited but the libraries included facilitate creating custom modules to integrate into a simulation. 

## Overview

Unlike generic simulation platforms which are often highly interpreted, UASISI modules are written directly in C++. This allows the framework more freedom in the modules that it can implement and specifically, it makes interacting with external systems more streamlined. For example, the pythonWrapper module, as the name suggests, supports the execution of python code in various formats which opens the door to easy machine learning exploration with well documented python packages, amongst a plethora of other ready-made libraries. Another example of a situation where UASIS could prove very useful is in coupling with external solvers such as openFOAM.

A UASISI simulation case is comprised by an orchestrator, a set of modules, and the appropriate communication channels linking everything together. The orchestrator is responsible for creating, configuring and connecting modules as well as manually initializing and stepping through each module. Orchestrators are instantiated and controlled from main `.cpp` files (see `/examples/simpleRun` for a practical example). Communication between modules is carried out via unique pointers to data structures owned by the orchestrator. 



## Features

- Specialized data structures for representing spanwise data
- Built in methods for processing spanwise data
- Generic and specialized module base classes
- Module registration system

## Architecture

<!-- Add later -->


## Modules


<!-- A table. The pipes and dashes are the syntax.
     The "|---|---|" line is required — it separates the header from the rows. -->

| **Name** | **Description** |
|------|-------------|
|   arbitraryOpt   | Arbitrary optimizer. A fixed lift distribution is manually set and the output will not change for the duration of the simulation            |
|------|-------------|
|   randomOpt   | Random optimizer. A random continous lift distribution is generated every specified amount of simulation time. Requires lift masks to ensure physically valid output (see `examples/reinforcementLearning` for an example)            |
|------|-------------|
|   constantCtrl   | Constant control. Outputs a specified and static control signal for the duration of the simulation         |
|------|-------------|
| perfectAct | Perfect actuators. Lets the control signal pass through unchanged. Can add actuator thickness if desired|
|------|-------------|
| perfectAct | Perfect actuators. Lets the control signal pass through unchanged. Can add actuator thickness if desired|
|------|-------------|
| constantAccelAct | Constant acceleration actuators. Models basic torsion actuator dynamics |
|------|-------------|
| phillipPhy | Phillips' physics. Implements Phillips' solution to Prandtl's LLT to approximate lift distribution |
|------|-------------|
| perfectSen | Perfect Sensor. Pass through module |
|------|-------------|
| simpleLogger | Connects to every available signal in the simulation and generates `.hd5` log files. A visualizer `tools/hd5viewer.py` is also provided|
|------|-------------|
| pythonWrapper | Allows interpreting python code and facilitating communication with other modules (see `examples/reinforcementLearning` for an example)|

## Dependencies

<!-- List what needs to be installed before building.
     **word** makes it bold. -->

**Required:**

- 
- 

**Optional:**

- 
- 

## Installation

**Must clone with `--recurse-submodules`:**
```bash
git clone --recurse-submodules https://github.com/Bel16ario/uasisi.git
cd uasisi
```

### Build

```bash
mkdir build && cd build
cmake ..
make
```

### Python Support (pythonWrapper)

## Project Structure


```
uasisi/
├── include/
├── src/
├── modules/
├── examples/
├── third_party/
├── tools/
└── CMakeLists.txt
```

## License

See LICENSE
