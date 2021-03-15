sudo docker pull debbieliang/ros_open:open_ros_dataset
sudo docker pull debbieliang/ros_open:open_ros_server

sudo docker run -d --network host --rm -it debbieliang/ros_open:open_ros_dataset roscore

sudo docker run -d --network host --rm -it debbieliang/ros_open:open_ros_server

#run this locally 
# sudo docker run --network host --rm -it debbieliang/ros_open:open_ros_dataset /bin/bash -c "source /openvslam/ros/devel/setup.bash && rosrun publisher video -m /openvslam/build/aist_living_lab_1/video.mp4"

sudo docker run -d --network host --rm -it debbieliang/ros_open:open_ros_latency /bin/bash -c "source /openvslam/ros/devel/setup.bash && rosrun openvslam run_slam -v /openvslam/build/orb_vocab/orb_vocab.dbow2 -c /openvslam/build/aist_living_lab_1/config.yaml --map-db map.msg"

