# perception-neuron-ros
This repository contains two ROS Modules: 
- a Windows ROS Serial Module, which sends the Perception Neuron BVH Data to a ROS Serial Server 
- a ROS BVH tf broadcaster package.

Usage 
=====
On your windows machine:
1) Connect your Perception Neuron to Axis Neuron and publish the BVH data.
2) Adapt IP addresses for ROS Serial Server and Axis Neuron in windows/config.txt 
3) Start windows/PerceptionNeuronROSSerial.exe 

On your ROS machine:
1) start roscore and ros serial server
2) start perception_neuron_tf_broadcaster

