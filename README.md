[![Build Status](https://travis-ci.org/tingelst/tpk4170-robotics.svg?branch=master)](https://travis-ci.org/tingelst/tpk4170-robotics)

# TPK4170 Robotics

Python code accompanying the course TPK4170 Robotics at the Department of Mechanical and Industrial Engineering (MTP) at the Norwegian University of Science and Technology (NTNU), Trondheim, Norway.

## Getting started

### Installation

`tpk4170` is a Python 3.6+ library. It can be installed with:
```bash
pip install tpk4170
```
Alternatively, you can clone the repository and run `pip install .` from inside the repository folder.

To use the 3-D visualization you also need to run:
```bash
jupyter nbextension install --py --symlink --sys-prefix pythreejs
jupyter nbextension enable --py --sys-prefix pythreejs
```
