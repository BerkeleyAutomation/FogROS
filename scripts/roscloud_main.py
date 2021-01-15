#!/usr/bin/python

import rospy
import re
import shutil
import rospkg
import boto3
from botocore.exceptions import ClientError
import os

#ROSCLOUD_DIR = "/home/ubuntu/catkin_ws/src/roscloud/" #os.path.dirname(os.path.realpath(__file__))
PRIV_KEY_PATH = "/home/ubuntu/a.pem"
ZIP_FILE_TMP_PATH = "/tmp"

def make_zip_file(dir_name, target_path):
    pwd, package_name = os.path.split(dir_name)
    print("zipped ", package_name)
    return shutil.make_archive(base_dir = package_name, root_dir = pwd, format = "zip", base_name = target_path)
    
if __name__ == '__main__':
    rospy.init_node('roscloud')

    ec2 = boto3.client('ec2', "us-west-1")
    # Do a dryrun first to verify permissions
    instance_id = "i-0830a57e084eb8799"
    #instance_dict = ec2.describe_instances().get('Reservations')[0]
    #print(instance_dict)
    
    '''
    ec2 = boto3.resource('ec2')
    instances = ec2.create_instances(
    ImageId='ami-f0091d91',
    MinCount=1,
    MaxCount=1,
    InstanceType='t2.micro',
    KeyName='<KEY-NAME>',
    SecurityGroups=['<GROUP-NAME>'])
    instance = instances[0]

    # Wait for the instance to enter the running state
    instance.wait_until_running()

    # Reload the instance attributes
    instance.load()
    print(instance.public_dns_name)

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

    launch_file = rospy.get_param('~launch_file', "")
    if not launch_file:
        print("has to have launch file!")
    with open(launch_file) as f:
        launch_text = f.read()
        _ , launch_file_name = os.path.split(launch_file)
        
    rospack = rospkg.RosPack()
    # find all the packages
    # package need to follow ros naming convention
    # i.e. flat namespace with lower case letters and underscore separators
    packages = set(re.findall(r"pkg=\"[a-z_]*\"" ,launch_text))
    packages.add("pkg=\"roscloud\"")
    print(packages)
    zip_paths = []
    
    for package in packages:
        package = package.split("\"")[1]
        print(package)
        pkg_path = rospack.get_path(package)
        print(pkg_path)
        zip_path = make_zip_file(pkg_path, "/tmp/" + package)
        print(zip_path)
        zip_paths.append(zip_path)


    ec2_resource = boto3.resource('ec2', "us-west-1")
    instance = ec2_resource.Instance(instance_id)
    public_ip = (instance.public_ip_address)
    print("public ip", public_ip)
    import paramiko
    from scp import SCPClient
    private_key = paramiko.RSAKey.from_private_key_file(PRIV_KEY_PATH)#"/home/keplerc/Downloads/ros-ec2.pem")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname = public_ip, username = "ubuntu", pkey = private_key )
    with SCPClient(ssh_client.get_transport()) as scp:
        for zip_file in zip_paths:
            scp.put(zip_file, '~/catkin_ws/src')
        #print('cd ~/catkin_ws/src && unzip -vo ' + ".zip ".join(package_names) + ".zip")

        # stdin, stdout, stderr = ssh_client.exec_command("cd ~/catkin_ws/src && for i in *.zip; do unzip -o \"$i\" -d \"${i%%.zip}\"; done " , get_pty=True)
        stdin, stdout, stderr = ssh_client.exec_command("cd ~/catkin_ws/src && for i in *.zip; do unzip -o \"$i\" -d . ; done " , get_pty=True)
        
        for line in iter(stdout.readline, ""):
            print(line, end="")

        scp.put(launch_file, "~/catkin_ws/src/roscloud/launch/" + launch_file_name)

        stdin, stdout, stderr = ssh_client.exec_command('cd ~/catkin_ws/ && source ./devel/setup.bash && catkin_make && roslaunch roscloud ' + launch_file_name , get_pty=True)

        for line in iter(stdout.readline, ""):
            print(line, end="")        
