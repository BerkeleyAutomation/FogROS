Note that this repository is for FogROS v1, which has been deprecated.

We are moving all of our development efforts to [FogROS2](https://github.com/berkeleyAutomation/FogROS2) which is based on ROS2 since ROS1 is being discontinued soon. 
=

# FogROS 

By Kaiyuan(Eric) Chen, Yafei Liang, Nikhil Jha, Jeffrey Ichnowski, Michael Danielczuk, Joseph Gonzalez, John Kubiatowicz, Ken Goldberg



## What is FogROS 

FogROS is a framework that allows existing ROS automation applications to gain access to additional computing resources from commercial cloud-based services. This framework is built on the [Robot Operating System (ROS)](https://www.ros.org/), the de-facto standard for creating robot automation applications and components. With minimal porting effort, FogROS allows researchers to deploy components of their software to the cloud with high transparency.

FogROS presents a user-friendly framework that allows computationally intensive parts of robotics programs (e.g., GPU, multi-core CPU, FPGA, TPU) to be run in the cloud; in experiments, common robotics algorithms (motion planning, grasp planning, VSLAM) deployed to the cloud via FogROS showed up to 31.5x performance improvement.The authors aim to make cloud-based deployments through FogROS transparent to researchers and roboticists. 

FogROS takes care of the details of setting up cloud computers, securing communication, and running programs in the cloud, allowing roboticists to benefit for the performance gain, allowing researchers to experiment with different deployments or focus on their application.With FogROS, researchers can deploy their software so that other researchers can make use of it with minimal effort and without concern for hardware compatibility--potentially allowing for increased cross-pollination of research ideas and more productive labs.

![architecture diagram depicting 4 nodes, one of which is in the cloud with FogROS](https://github.com/BerkeleyAutomation/FogROS/raw/main/docs/FogROS.gif)

This project will be published in [IEEE International Conference on Automation Science and Engineering (CASE) 2021](https://case2021.sciencesconf.org/). The video can be found on [Youtube](https://www.youtube.com/watch?v=lSZw_Fkpnm0&t=2s), and the paper can be found [Here](https://github.com/BerkeleyAutomation/FogROS/blob/main/docs/FogROS_CASE2021_Camera%20Ready.pdf). 

## Install

#### Step 0: Set up ROS
Set up ROS environment and clone this repo to catkin workspace. ```ami-0d255df33d23c5a9d``` in ```US-west-1``` is a good starting point if you don't have a working ROS environment. 


#### Step 1:  Set up the repo's environments

1. you need to run ```aws configure``` to configure the AWS API keys. 
2. ```launch/roscloud*.launch``` has plenty of options. You may specify a docker image, or just several ROS nodes. The code for the demos can be found in [Demo Repo](https://github.com/BerkeleyAutomation/fogros-demos). 

#### Step2: Run it 
Simply run 

````
roslaunch FogROS roscloud_talker.launch 
````

for basic pubisher-subscriber example. The EC2 instance will be automatically created, and you will verify the code is working by running 
```
rostopic list
```
on your robot or local machine. The step-by-step tutorial can be found in our [Wiki](https://github.com/BerkeleyAutomation/FogROS/wiki/Running-FogROS-Examples)



## Release Plan 

Currently, FogROS is still a research prototype. It is going through code and documentation revisions. We are planning to have official release by mid-September. 



## Contact Us 

If there were any concerns/bugs, please post as Github issue. 
