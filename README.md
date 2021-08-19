# FogROS 

By Kaiyuan(Eric) Chen, Yafei Liang, Nikhil Jha, Jeffrey Ichnowski, Michael Danielczuk, Joseph Gonzalez, John Kubiatowicz, Ken Goldberg



## What is FogROS 

FogROS is a framework that allows existing ROS automation applications to gain access to additional computing resources from commercial cloud-based services. This framework is built on the Robot Operating System (ROS), the de-facto standard for creating robot automation applications and components. With minimal porting effort, FogROS allows researchers to deploy components of their software to the cloud with high transparency.

The paper presents a user-friendly framework that allows computationally intensive parts of robotics programs (e.g., GPU, multi-core CPU, FPGA, TPU) to be run in the cloud; in experiments, common robotics algorithms (motion planning, grasp planning, VSLAM) deployed to the cloud via FogROS showed up to 31.5x performance improvement.The authors aim to make cloud-based deployments through FogROS transparent to researchers and roboticists--FogROS takes care of the details of setting up cloud computers, securing communication, and running programs in the cloud, allowing roboticists to benefit for the performance gain, allowing researchers to experiment with different deployments or focus on their application.With FogROS, researchers can deploy their software so that other researchers can make use of it with minimal effort and without concern for hardware compatibility--potentially allowing for increased cross-pollination of research ideas and more productive labs.



## Install

#### Step 0: Set up ROS
Set up ROS environment and clone this repo to catkin workspace. ```ami-0d7962a5f43faec05``` in ```US-west-1``` is a good starting point if you don't have a working ROS environment. 


#### Step 1:  Set up the repo's environments

1. you need to run ```aws configure``` to configure the AWS API keys. 
2. ```launch/roscloud*.launch``` has plenty of options. You may specify a docker image, or just several ROS nodes 

#### Step2: Run it 
Simply run 

````
roslaunch roscloud roscloud.launch 
````

The EC2 instance will be automatically created, and you will verify the code is working by running 
```
rostopic list
```
on your robot or local machine. 



### Release Plan 

Currently, FogROS is still a research prototype. It is going through code and documentation revisions. We are planning to have official release by mid-September. 



### Contact Us 

If there were any concerns, please post as Github issue, or send an email to Eric Chen by kych@berkeley.edu. 
