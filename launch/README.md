# Launch Scripts 

### How to use 

We will run ```roscloud_*``` on the client, and the script will automatically upload its counterpart to the cloud. For example, if we run

```bash
roslaunch roscloud roscloud_talker.launch
```
It will start ```roscloud/talker.py``` node on the cloud instance. 


### List of files
The following is a list of files that can be run with ```roslaunch roscloud```
* roscloud_talker.launch: it starts a basic talker node on the cloud, to test whether the traffic is relayed to the robot, one can run ```rosrun roscloud listener.py``` on the robot, or simply use ```rostopic echo /chatter```. 
* roscloud_mpt.launch: it starts a motion planner template on the cloud
* roscloud_gqcnn_launch.launch: it starts a dexnet node on the cloud 
* roscloud_openvslam.launch: it starts an oepnvslam node on the cloud 
* roscloud_vpc_: are the counterparts that use vpc  
