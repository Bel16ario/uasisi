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

## Modules


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
|------|-------------|

## Dependencies

**Required:**

- C++17
- CMake 3.15+
- Eigen3 3.3+
- GSL
- HDF5

**As submodules**

- yaml-cpp
- HighFive

**For use with python:**

- Python 3.8+
- pybind11

**For python h5 viewer**

- h5py
- numpy
- matplotlib

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

Python is supported through the pythonWrapper module. A template `.py` file is provided in `/modules/modules/pythonWrapper/template.py`. This wrapper can execute code from scripts or string literals. KWArgs can passed through an auxiliary PythonConfigDict class. Communication between the base C++ layer and the python interpreter can be carried out through pybind11 embedded python objects as well as the provided input and output dictionary translation methods. To interact with the rest of the modules, a main `.py` script can be provided with defined functions `step(t, dt, inputs)` and `init(inputs)`. In this case, `inputs` is a dictionary containing signal data and both functions should return a corresponding `outputs` signal dictionary. Additional kwargs can be implemented as needed. The `reinforcementLearning` example provided in this repo is a good example of how to integrate python code into UASISI.

It should be noted that UASISI does not yet support multi-threading and as such, only one python interpreter can be alive at any given moment. In practice this means that users should be careful when implementing multiple pythonWrapper modules as the scope might be shared between them.

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
