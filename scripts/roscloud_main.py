#!/home/keplerc/anaconda3/bin/python

import rospy
import re
import shutil
import rospkg


def make_zip_file(dir_name, target_path):
    print(shutil.make_archive(base_dir = dir_name, root_dir = dir_name, format = "zip", base_name = target_path))
    
if __name__ == '__main__':
    rospy.init_node('roscloud')
    launch_file = rospy.get_param('~launch_file', "")
    if not launch_file:
        print("has to have launch file!")
    with open(launch_file) as f:
        launch_text = f.read()

    rospack = rospkg.RosPack()
    # find all the packages
    # package need to follow ros naming convention
    # i.e. flat namespace with lower case letters and underscore separators
    packages = set(re.findall(r"pkg=\"[a-z_]*\"" ,launch_text))
    for package in packages:
        package = package.split("\"")[1]
        print(package)
        pkg_path = rospack.get_path(package)
        print(pkg_path)
        make_zip_file(pkg_path, "/home/keplerc/catkin_ws/src/roscloud/"+package)
    
