# FogROS 



## What is FogROS 

RosCloud enables ROS application developers to easily offload their ROS applications to Amazon EC2 servers. After the initial setup(AWS related), developers simply start an offload request node and the projects will automatically run on the remote server. Example of the initiating script is ```launch/roscloud*.launch```.



## How to set it up

#### Step 0: Set up ROS
Set up ROS environment and clone this repo to catkin workspace. ```ami-05829bd3e68bcd415``` is a good starting point. 


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

