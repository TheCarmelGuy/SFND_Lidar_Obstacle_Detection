# Sensor Fusion Self-Driving Car Course: Lidar Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />


## Description

This module does full-fledged Lidar clustering. The submodules include:

1. Planar RANSAC to filter our ground plane
2. Euclidean Clustering for obstacle detection using Custom-built KD tree
3. PCA bouding Boxes fitting on clusters to determine pose

## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

