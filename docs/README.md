# FogROS: An Adaptive Framework for Automating Fog Robotics Deployment

FogROS is a framework that allows existing ROS automation applications to gain access to additional computing resources from commercial cloud-based services. This framework is built on the [Robot Operating System (ROS)](https://www.ros.org/), the de-facto standard for creating robot automation applications and components. With minimal porting effort, FogROS allows researchers to deploy components of their software to the cloud with high transparency.

FogROS presents a user-friendly framework that allows computationally intensive parts of robotics programs (e.g., GPU, multi-core CPU, FPGA, TPU) to be run in the cloud; in experiments, common robotics algorithms (motion planning, grasp planning, VSLAM) deployed to the cloud via FogROS showed up to 31.5x performance improvement.The authors aim to make cloud-based deployments through FogROS transparent to researchers and roboticists. 

FogROS takes care of the details of setting up cloud computers, securing communication, and running programs in the cloud, allowing roboticists to benefit for the performance gain, allowing researchers to experiment with different deployments or focus on their application.With FogROS, researchers can deploy their software so that other researchers can make use of it with minimal effort and without concern for hardware compatibility--potentially allowing for increased cross-pollination of research ideas and more productive labs.

![architecture diagram depicting 4 nodes, one of which is in the cloud with FogROS](https://github.com/BerkeleyAutomation/FogROS/raw/main/docs/FogROS.gif)

This project will be published in [IEEE International Conference on Automation Science and Engineering (CASE) 2021](https://case2021.sciencesconf.org/). The video can be found on [Youtube](https://www.youtube.com/watch?v=lSZw_Fkpnm0&t=2s), and the paper can be found [Here](https://github.com/BerkeleyAutomation/FogROS/blob/main/docs/FogROS_CASE2021_Camera%20Ready.pdf).
