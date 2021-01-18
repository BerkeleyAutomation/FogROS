#!/usr/bin/python

import rospy
import re
import shutil
import rospkg
import boto3
from botocore.exceptions import ClientError
import os
import paramiko
from scp import SCPClient


# private key path for the EC2 instance
PRIV_KEY_PATH = "/home/ubuntu/a.pem"

def make_zip_file(dir_name, target_path):
    pwd, package_name = os.path.split(dir_name)
    return shutil.make_archive(base_dir = package_name, root_dir = pwd, format = "zip", base_name = target_path)

if __name__ == '__main__':
    rospy.init_node('roscloud')
    TO_CLOUD_LAUNCHFILE_NAME = rospy.get_param('~to_cloud_launchfile_name', "to_cloud.launch")
    MY_IP_ADDR = rospy.get_param('~rosbridge_ip_addr')
    PRIV_KEY_PATH = rospy.get_param('~private_key_path')
    ZIP_FILE_TMP_PATH = rospy.get_param('~temporary_dir', "/tmp")
    
    image_id = rospy.get_param('~ec2_instance_image', 'ami-05829bd3e68bcd415')
    ec2_instance_type = rospy.get_param('~ec2_instance_type', 't2.micro')
    # name of existing key pair
    # TODO: get a new one if this paramter is not there
    ec2_key_name = ros.get_param('~ec2_key_name')
    ec2_security_group_ids = ros.get_param('~ec2_security_group_ids', [])
    
    exit()
    
    ec2_resource = boto3.resource('ec2', "us-west-1")
    instances = ec2_resource.create_instances(
        ImageId=image_id,
        MinCount=1,
        MaxCount=1,
        InstanceType=ec2_instance_type,
        KeyName= ec2_key_name,
        SecurityGroupIds= ec2_security_group_ids
    )
    print("Have created the instance: ", instances)
    instance = instances[0]

    #instance_dict = ec2.describe_instances().get('Reservations')[0]
    #print(instance_dict)
    
    '''
    # Wait for the instance to enter the running state
    instance.wait_until_running()

    # Reload the instance attributes
    instance.load()
    print(instance.public_dns_name)

    ec2 = boto3.client('ec2', "us-west-1")
    try:
        ec2.start_instances(InstanceIds=[instance_id], DryRun=True)
    except ClientError as e:
        if 'DryRunOperation' not in str(e):
            raise

    try:
        response = ec2.start_instances(InstanceIds=[instance_id], DryRun=False)
        print(response)
    except ClientError as e:
        print(e)
    '''

    launch_file = rospy.get_param('~launch_file')
    
    with open(launch_file) as f:
        launch_text = f.read()
        launch_file_dir , launch_file_name = os.path.split(launch_file)

    with open(launch_file_dir + "/" + TO_CLOUD_LAUNCHFILE_NAME , "w") as f:
        f.write(launch_text.replace("ROSBRIDGE_IP_ADDR_REPLACE", MY_IP_ADDR))
        
    # find all the packages
    # package need to follow ros naming convention
    # i.e. flat namespace with lower case letters and underscore separators
    # then zip all the packages
    rospack = rospkg.RosPack()
    packages = set(re.findall(r"pkg=\"[a-z_]*\"" ,launch_text))
    packages.add("pkg=\"roscloud\"")
    print(packages)
    zip_paths = []
    for package in packages:
        package = package.split("\"")[1]
        pkg_path = rospack.get_path(package)
        zip_path = make_zip_file(pkg_path, "/tmp/" + package)
        zip_paths.append(zip_path)


    # get public ip address of the EC2 server

    #instance_id = "i-0830a57e084eb8799"
    #instance = ec2_resource.Instance(instance_id)
    public_ip = instance.public_ip_address

    # start a SSH/SCP session to the EC2 server 
    private_key = paramiko.RSAKey.from_private_key_file(PRIV_KEY_PATH)#"/home/keplerc/Downloads/ros-ec2.pem")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname = public_ip, username = "ubuntu", pkey = private_key )
    
    with SCPClient(ssh_client.get_transport()) as scp:
        # transfer all the zip files to the EC2 server's workspace 
        for zip_file in zip_paths:
            scp.put(zip_file, '~/catkin_ws/src')

        # use SSH to unzip them 
        stdin, stdout, stderr = ssh_client.exec_command("cd ~/catkin_ws/src && for i in *.zip; do unzip -o \"$i\" -d . ; done " , get_pty=True)
        
        for line in iter(stdout.readline, ""):
            print(line, end="")

        # transfer the launch script 
        scp.put(launch_file_dir + TO_CLOUD_LAUNCHFILE_NAME, "~/catkin_ws/src/roscloud/launch/" + TO_CLOUD_LAUNCHFILE_NAME)

        # roslaunch the script on EC2 instance 
        stdin, stdout, stderr = ssh_client.exec_command('cd ~/catkin_ws/ && source ./devel/setup.bash && catkin_make && roslaunch roscloud ' + TO_CLOUD_LAUNCHFILE_NAME , get_pty=True)

        for line in iter(stdout.readline, ""):
            print(line, end="")        
