# MANO Directory

This directory contains the MANO (hand Model with Articulated and Non-rigid defOrmations) hand model files and utilities.

## Files

- **__init__.py** - Python package initialization file
- **LICENSE.txt** - License information for MANO model

## Subdirectories

### models/
Contains the MANO model data files:
- **info.txt** - Information about the model files
- **LICENSE.txt** - Model-specific license

### webuser/
Python implementation and utilities for MANO model:
- **__init__.py** - Package initialization
- **lbs.py** - Linear Blend Skinning implementation
- **LICENSE.txt** - Code license
- **posemapper.py** - Pose mapping utilities
- **README.txt** - Documentation
- **serialization.py** - Model serialization/deserialization
- **smpl_handpca_wrapper_HAND_only.py** - SMPL hand PCA wrapper (hand only)
- **smpl_handpca_wrapper.py** - SMPL hand PCA wrapper (full)
- **verts.py** - Vertex manipulation utilities

### hello_world/
Example scripts demonstrating MANO usage:
- **__init__.py** - Package initialization
- **MANO___hello_world.py** - Basic MANO usage example
- **MANO___render.py** - MANO rendering example
- **SMPL+H___hello_world.py** - SMPL+H basic example
- **SMPL+H___render.py** - SMPL+H rendering example

MANO is a parametric hand model widely used in computer vision and graphics for hand pose estimation and reconstruction.
