
sudo apt install -y docker.io
sudo docker pull keplerc/grasp_image:gpu
sudo docker run --gpus all -d --network host --rm keplerc/grasp_image:gpu 
# sleep 10
# sudo docker run --network host --rm keplerc/grasp_image:gpu roslaunch gqcnn_ros gqcnn_ros_client.launch client_args:=" --camera_intr=/root/catkin_ws/src/gqcnn-ros/data/calib/phoxi.intr --depth_image=/root/catkin_ws/src/gqcnn-ros/data/examples/phoxi_clutter/depth_0.npy"

