PCL Spectrolab
===========================
by Adam Stambler

This repository contains the code from the 
[Spectrolab PCL code sprint](  www.pointclouds.org/blog/spectrolab/ ).
Development of this software was sponsored by [Spectrolab]( www.spectrolab.com/) and 
[Open Perception](www.openperception.org).  It contains a [Spectroscan 3D]
(http://www.spectrolab.com/sensors/pdfs/products/SPECTROSCAN3D_RevA%20071912.pdf) driver,
PCL Grabber, and 3D Viewer.

The latest version of this repository can be found on :
https://github.com/SpectrolabInc/spectrolab

## Installation

The software requires PCL and Qt4.  You can find precompiled binaries [here](http://pointclouds.org/downloads/)
To compile, go into the pcl_spectrolab repository directory

	mkdir build
	cd buid
	cmake .. & make

##Setup
To use the software, the computer must be connected to the scanner
such that the computer has the IP address 192.168.0.121.  You need
to specify how to find the scanner using arp

Unix  :
	sudo arp -s 192.168.0.27 00:0F:CC:23:00:01
Windows  : 
	arp -s 192.168.0.27  00-0F-CC-23-00-01

This needs to be done everytime the system is started.

All programs that use the scanner must currently be run as Administrator 
or root user because the system uses low port numbers.

To view a live point cloud stream, record the stream, or play back a saved stream:

	sudo ./spectrolab_viewer

To test the scanner, you can grab a single frame using 

	sudo ./spectroscan3d_grabpcd  frame


##Spectrolab Viewer 

The Spectrolab Viewer is 3D point cloud viewer which can load .pcd 
(point cloud data) and .ssi (Spectrolab Scan Images).  The point clouds 
can be colored by many different renderes for optimal viewing. Choose renders
by using the viewer's play button menu.  Additionally, 
the viewer can open directories of scans and play them back as a streaming
movie.

The viewer also doubles as a live streaming interface to the Spectroscan
3D lidar camera.  Use the Spectroscan3D menu to connect and interact with
the scanner. 

To record Spectroscan3D movies, simply use the record button on the player.
Movies can be recorded as PCD or SSI files.

	sudo ./spectrolab_viewer


##Example Programs 
The tools directory contains basic programs demonstrating the driver's 
API.  Use these programs to learn how to use this repository.
There are example programs for grabing point clouds, save intensity
images, and creating a simple streaming viewer for your Spectroscan 3D.

* spectroscan3d_grabframe.cpp -  grab a frame and save as an ssi binary
* spectroscan3d_grabpcd.cpp   -  grab a frame and save as a pcd file
* spectroscan3d_frame_to_png.cpp - convert a Spectroscan 3D ssi binary to a png intensity image
* simple_movie_player.cpp -  play a directory of *.ssi binary files in a 3D viewer
* spectroscan3d_simple_viewer.cpp - stream a point cloud from a Spectroscan 3D to a 3D viewer
 
 
