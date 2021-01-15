# RosCloud 



## What is RosCloud 

RosCloud enables ROS application developers to easily offload their ROS applications to Amazon EC2 servers. After the initial setup(AWS related), developers simply start an offload request node and the projects will automatically run on the remote server. Example of the initiating script is ```launch/roscloud.launch``` and the uploaded script is ```launch/myscript.launch```. 



## How to set it up

#### Step 0: run rosduct-baseline 

To fully understand what's happening, you need to run **rosduct-baseline** tutorial successfully. Or you can start an EC2 instance with image ```ami-05829bd3e68bcd415```  and do the following steps from there. 



#### Step 1:  Set up the repo's environments

There are plenty of variables to be set correctly before running the project. 

1. ```launch/roscloud.launch``` has a path to the launchfile that needs to be run on the cloud. It has to be an absolute path. 
2. AWS related. You need to run ```aws configure``` to configure the AWS API keys. You also need to set up a security group to allow ros traffic(which should be available if you have done step 0). 
3. ```script/rescloud_main.py``` has several variables such as ```MY_IP_ADDR```, ```PRIV_KEY_PATH```  ; AWS related parameters such as ```SecurityGroupIds``` and ```KeyName```. 

TODO: we need to reduce and centralize the variables needed. 


#### Step2: Run it 

First run rosbridge server by 

```
roslaunch rosbridge_server rosbridge_websocket.launch
```



Then  run 

````
roslaunch roscloud roscloud.launch 
````

The EC2 instance will be automatically created, and you will see the console printing "I heared Hello World" sent by the remote EC2 server. 

