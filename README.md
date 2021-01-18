# RosCloud 



## What is RosCloud 

RosCloud enables ROS application developers to easily offload their ROS applications to Amazon EC2 servers. After the initial setup(AWS related), developers simply start an offload request node and the projects will automatically run on the remote server. Example of the initiating script is ```launch/roscloud.launch``` and the uploaded script is ```launch/myscript.launch```. 



## How to set it up

#### Step 0: run rosduct-baseline 

To fully understand what's happening, you need to run or at least play with **rosduct-baseline** tutorial. You can start an EC2 instance with image ```ami-05829bd3e68bcd415```  and start from there. 


#### Step 1:  Set up the repo's environments

```launch/roscloud.launch``` has plenty of variables to be set correctly before running the project. 

1. It has a ```launch_file``` which is the path to the launchfile that needs to be run on the cloud. It has to be an absolute path. 
2. AWS related. Before changing the launch file, you need to run ```aws configure``` to configure the AWS API keys. You also need to generate AWS key pair and set up a security group to allow the traffic to your host machine(these should be available after you have done step 0). 
3. It also has other variables such as rosbridge's IP address and the path of EC2 private key(used to set up the EC2 instance)


#### Step2: Run it 
Simply run 

````
roslaunch roscloud roscloud.launch 
````

The EC2 instance will be automatically created, and you will see the console printing "I heared Hello World" sent by the remote EC2 server. 


### Aside

I am still working on code optimizations, so the current commit is NOT TESTED. If any of the code doesn't work, first checkout commit 272a60f. (which I have tagged that commit to make it easier to find)


