
sudo apt install -y docker.io
sudo docker pull keplerc/grasp_image:cpu
sudo docker run -d --network host --rm keplerc/grasp_image:cpu
sudo docker run --network host --rm keplerc/grasp_image:cpu rosrun gqcnn-ros gqcnn_ros_client.py --camera_intr=/root/catkin_ws/src/gqcnn-ros/data/calib/phoxi.intr --depth_image=/root/catkin_ws/src/gqcnn-ros/data/examples/phoxi_clutter/depth_0.npy
