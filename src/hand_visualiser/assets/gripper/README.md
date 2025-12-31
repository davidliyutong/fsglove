# Gripper Directory

This directory contains robotic gripper models and descriptions, specifically for the Shadow Hand robot.

## Files

- **__init__.py** - Python package initialization file
- **shadow.urdf** - Main URDF (Unified Robot Description Format) file for the Shadow Hand

## Subdirectories

### shadow/
Contains the complete Shadow Hand grasp description package:

- **sr_grasp_description/** - Shadow Robot grasp description package
  - **meshes/** - 3D mesh files for the robot components
  - **urdf/** - Additional URDF files and Xacro templates
    - `gazebo_shadowhand_standalone.urdf.xacro` - Gazebo simulation configuration
    - `shadowhand_motor_ellipsoid.orig.urdf` - Original motor configuration
    - `shadowhand.urdf` - Main hand description
  - **simox/** - Simox simulator configuration
    - `shadowhand.xml` - Main configuration file
    - **model/** - 3D models in various formats (WRL, IV)
      - Includes models for fingers (F2, F3), thumb (TH2_z, TH3_z), palm, wrist, and other components
  - **contrib/** - Contributed utilities
    - `fix_urdf.py` - URDF fixing script

This package provides the complete robot model for simulation and visualization purposes.
