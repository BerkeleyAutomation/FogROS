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
from io import StringIO
from requests import get
import time
import random

# ec2 console coloring 
CRED = '\033[91m'
CEND = '\033[0m'

aws_region = "us-west-1"

def make_zip_file(dir_name, target_path):
    pwd, package_name = os.path.split(dir_name)
    return shutil.make_archive(base_dir = package_name, root_dir = pwd, format = "zip", base_name = target_path)

def aws_create_security_group(ec2, ec2_security_group):
    response = ec2.describe_vpcs()
    vpc_id = response.get('Vpcs', [{}])[0].get('VpcId', '')

    try:
        response = ec2.create_security_group(GroupName=ec2_security_group,
                                         Description='DESCRIPTION',
                                         VpcId=vpc_id)
        security_group_id = response['GroupId']
        print('Security Group Created %s in vpc %s.' % (security_group_id, vpc_id))

        data = ec2.authorize_security_group_ingress(
                GroupId=security_group_id,
                IpPermissions=[
                    {'IpProtocol': '-1',
                     'FromPort': 0,
                     'ToPort': 65535,
                     'IpRanges': [{'CidrIp': '0.0.0.0/0'}]
                    }
                ])
        print('Ingress Successfully Set %s' % data)
        ec2_security_group_ids = [security_group_id]
    except ClientError as e:
        print(e)
    print("security group id is " + str(ec2_security_group_ids))
    return ec2_security_group_ids

def aws_generate_key_pair(ec2, ec2_key_name):
    ec2_keypair = ec2.create_key_pair(KeyName=ec2_key_name) 
    ec2_priv_key = ec2_keypair['KeyMaterial']
    with open("/home/ubuntu/" + ec2_key_name + ".pem", "w") as f:
        f.write(ec2_priv_key)
    print(ec2_priv_key)
    return ec2_priv_key


def aws_create_instance(ec2_resource, ec2_key_name, ec2_security_group_ids, ec2_instance_type="t2.large", ec2_instance_disk_size = 8):
    #
    # start EC2 instance
    # note that we can start muliple instances at the same time
    #
    instances = ec2_resource.create_instances(
        ImageId= 'ami-0099f9139c84a5007',  # 'ami-0757bbcb3ba382f34', #'ami-0099f9139c84a5007', # ImageId='ami-05829bd3e68bcd415',
        MinCount=1,
        MaxCount=1,
        InstanceType=ec2_instance_type,
        KeyName= ec2_key_name,
        SecurityGroupIds= ec2_security_group_ids,
        BlockDeviceMappings=[
            {
                'DeviceName': '/dev/sda1',
                'Ebs': {
                    'VolumeSize': 30,
                    'VolumeType': 'standard'
                }
            }
        ]
    )
    print("Have created the instance: ", instances)
    print("type: " + ec2_instance_type)
    instance = instances[0]
    # use the boto3 waiter
    print("wait for launching to finish")
    instance.wait_until_running()
    print("launch finished")
    # reload instance object
    instance.reload()
    #instance_dict = ec2.describe_instances().get('Reservations')[0]
    #print(instance_dict)
    return instance 

def prepare_launch_file(launch_file, magic_int, modify_launch=True):

    # 
    # read in the launchfile 
    # we also modify the launchfile IP address to this machine's public IP address
    # 
    launch_file_dir , launch_file_name = os.path.split(launch_file)
    with open(launch_file) as f:
        launch_text = f.read()

    my_ip = get('https://checkip.amazonaws.com').text.strip() 
    print("robot public address is ", my_ip)

    rosduct_launch_text = '''
    <node pkg="roscloud" name="rosduct" type="rosduct_main.py" output="screen">
    <rosparam>
        rosbridge_ip: ''' + my_ip + '''
        rosbridge_port: 9090
    </rosparam>
    </node>
</launch>
    '''
    with open("/tmp/to_cloud" + magic_int + ".launch" , "w") as f:
        if ("rosduct" not in launch_text) and modify_launch :
            f.write(launch_text.replace("</launch>", rosduct_launch_text))
        else:
            f.write(launch_text.replace("ROSBRIDGE_IP_HOLDER", my_ip))
        
    # find all the ROS packages in the launchscript
    # package need to follow ros naming convention
    # i.e. flat namespace with lower case letters and underscore separators
    # then zip all the packages
    # currently we assume all the packages can be catkin_make 
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
    return zip_paths 


def create_ec2_pipeline(rand_int, ec2_instance_type = "t2.large", image_id = "ami-05829bd3e68bcd415"):
    ec2_key_name = "foo" + rand_int
    ec2_security_group_name = 'SECURITY_GROUP_NAME' + rand_int
    ec2_resource = boto3.resource('ec2', aws_region)
    ec2 = boto3.client('ec2', aws_region)
    ec2_priv_key = aws_generate_key_pair(ec2, ec2_key_name)
    ec2_security_group_ids = aws_create_security_group(ec2, ec2_security_group_name)
    instance = aws_create_instance(ec2_resource, ec2_key_name, ec2_security_group_ids, ec2_instance_type)
    public_ip = instance.public_ip_address
    while not public_ip:
        instance.reload()
        public_ip = instance.public_ip_address
    print("public ip of ec2: " + public_ip)
    return public_ip, ec2_key_name



    
def connect_and_launch(ec2_key_name, zip_paths, public_ip, launch_file_dir, env_script, magic_int, env_command= ""):
    private_key = paramiko.RSAKey.from_private_key_file("/home/ubuntu/" + ec2_key_name + ".pem")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_client.connect(hostname = public_ip, username = "ubuntu", pkey = private_key, look_for_keys=False )

    with SCPClient(ssh_client.get_transport()) as scp:
        # transfer all the zip files to the EC2 server's workspace 
        for zip_file in zip_paths:
            scp.put(zip_file, '~/catkin_ws/src')

        # use SCP to upload the launch script
        # TODO: there might be a collision if we run multiple nodes, need to solve it 
        scp.put("/tmp/to_cloud" + magic_int + ".launch", "~/catkin_ws/src/roscloud/launch/to_cloud.launch")

        scp.put(env_script, "~/setup.bash")

            
        # use SSH to unzip them to the catkin workspace
        stdin, stdout, stderr = ssh_client.exec_command("cd ~/catkin_ws/src && for i in *.zip; do unzip -o \"$i\" -d . ; done " , get_pty=True)

        for line in iter(stdout.readline, ""):
            continue
            #print(CRED + line + CEND, end="")
        
        # execute setup script
        stdin, stdout, stderr = ssh_client.exec_command("chmod +x ~/setup.bash && ~/setup.bash" , get_pty=True)

        for line in iter(stdout.readline, ""):
            print(CRED + line + CEND, end="")


        # catkin_make all the uploaded packages
        # roslaunch the script on EC2  
        stdin, stdout, stderr = ssh_client.exec_command('cd ~/catkin_ws/ && source ./devel/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release' , get_pty=True)

        for line in iter(stdout.readline, ""):
            print("EC2: " + str(time.time()) + " " + CRED + line + CEND, end="")        


        stdin, stdout, stderr = ssh_client.exec_command(env_command + " echo $ROS_HOSTNAME && echo $ROS_MASTER_URI", get_pty=True)

        for line in iter(stdout.readline, ""):
            print("==============" + line, end="")        
        print("=================")
        print(stderr)
        print(env_command + " roslaunch roscloud to_cloud.launch")
        
        stdin, stdout, stderr = ssh_client.exec_command("source ~/catkin_ws/devel/setup.bash && " + env_command + " roslaunch roscloud to_cloud.launch", get_pty=True)

        for line in iter(stdout.readline, ""):
            print("EC2: " + str(time.time()) + " " + CRED + line + CEND, end="")        



    
def push_launch(launch_file, ec2_instance_type, env_script):
    rand_int = str(random.randint(10, 1000))
    print("start launching " + str(time.time()))
    public_ip, ec2_key_name =create_ec2_pipeline(rand_int, ec2_instance_type)
    if(rospy.get_name() == "/leader"):
        with open("/tmp/leader_info", "w+") as f:
            f.write("{}".format(public_ip))
    else:
        #TODO: depends on the task
        time.sleep(60)

    launch_file_dir , launch_file_name = os.path.split(launch_file)
    zip_paths = prepare_launch_file(launch_file, rand_int, False)
    time.sleep(60)

    leader_ip = ""
    with open("/tmp/leader_info") as f:
        leader_ip = f.read()
    with open(env_script) as f:
        env_script_txt = f.read()
        env_command = '''
export ROS_HOSTNAME={}
export ROS_MASTER_URI=http://{}:11311
'''.format(public_ip, leader_ip.strip())
        env_script_text = env_command + env_script_txt
        if (public_ip != leader_ip):
            print("need to modify the ip env")
            env_script_text = env_script_text.replace("docker run", "docker run -e ROS_HOSTNAME={}  -e ROS_MASTER_URI=http://{}:11311  ".format(public_ip, leader_ip.strip()))
            
    env_script = "/tmp/setup" + rand_int + ".bash"
    with open(env_script, "w+") as f:
        print(env_script_text)
        f.write(env_script_text)

    if public_ip == leader_ip:
        print("public == leader ===============")
        connect_and_launch(ec2_key_name, zip_paths, public_ip, launch_file_dir, env_script, rand_int)
    else:
        print("public != leader===========")
        connect_and_launch(ec2_key_name, zip_paths, public_ip, launch_file_dir, env_script, rand_int, "export ROS_HOSTNAME={} && export ROS_MASTER_URI=http://{}:11311 &&".format(public_ip, leader_ip.strip()))
        
def push_docker(docker_image, ec2_instance_type):
    rand_int = str(random.randint(10, 1000))
    print("start launching " + str(time.time()))
    docker_str ='''
sudo apt install -y docker.io
sudo docker pull ''' + docker_image + '''
sudo docker run -d --network host --rm ''' + docker_image

    launch_file_str = '''
<launch>
</launch>
'''
    launch_file = "/tmp/docker.launch"
    env_script = "/tmp/docker.bash"
    with open(launch_file, "w") as f:
        f.write(launch_file_str)
    public_ip, ec2_key_name =create_ec2_pipeline(rand_int, ec2_instance_type)
    launch_file_dir , launch_file_name = os.path.split(launch_file)
    zip_paths = prepare_launch_file(launch_file)
    with open("/tmp/docker.bash", "w") as f:
        f.write(docker_str)
    
    time.sleep(60)
    connect_and_launch(ec2_key_name, zip_paths, public_ip, launch_file_dir, env_script)

