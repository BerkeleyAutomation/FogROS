
sudo apt install -y docker.io
sudo docker pull keplerc/grasp_image:cpu
sudo docker run -d --network host --rm keplerc/grasp_image:cpu
