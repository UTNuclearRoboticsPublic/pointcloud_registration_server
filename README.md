# Table of Contents
1. [About](#about)
2. [Parameter Setup](#parameter-setup)
3. [Usage](#usage)

## About
This package provides a convenient ROS wrapper for the [Pointcloud Library](http://pointclouds.org/) registration libraries. It isolates the user from needing to interact directly with a PCL interface, instead providing a ROS service-based interface. So far, two registration algorithms are implemented - ICP and NDT. These can be used with various settings to register and concatenate an arbitrary number of input clouds.

The user can set pre-registration settings like voxelizations and spatial clipping to improve speed, and also independently control the respective output settings. That is, input clouds can be downsampled for registration, and the found transform can then be used to concatenate the fully-dense input clouds for the output.

<img src=images/registration.png width="600">

In the above example, two pointclouds were taken using the [laser_stitcher](https://github.com/UTNuclearRoboticsPublic/laser_stitcher) package from two slighlty different vantage points. Artificial errors were introduced to one cloud in position and rotation, and registration was performed between the clouds to correct this error and stitch them into a single pointcloud map of the scene. 

## Parameter Setup

## Usage
