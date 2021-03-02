#!/usr/bin/env python3

from genpy.message import fill_message_args
from pydoc import locate
import json
import yaml

"""
Conversions to-from ROS messages, JSON and dict
representations of them.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


def get_ROS_msg_type(ros_msg):
    """
    Returns the ROS message type as package_msgs/Message.
    :param AnyMsgInstance ros_msg: ROS message instance to
        get the type from.
    :returns str: ROS message type as package_msg/Message.
    """
    return ros_msg._type


def is_ros_message_installed(ros_message_type):
    """
    Return true if the message is found installed.
    False otherwise.
    :param str ros_message_type: ROS message type as package_msgs/Message.
    :returns bool: True if is installed, False otherwise.
    """
    try:
        package_name, message_name = ros_message_type.split('/')
    except ValueError:
        raise ValueError(
            'ros_message_type should be in the shape of package_msgs/Message' +
            ' (it was ' + ros_message_type + ')')
    msg_class = locate('{}.msg.{}'.format(package_name, message_name))
    if msg_class is None:
        return False
    else:
        return True


def is_ros_service_installed(ros_message_type):
    """
    Return true if the message is found installed.
    False otherwise.
    :param str ros_message_type: ROS message type as package_msgs/Message.
    :returns bool: True if is installed, False otherwise.
    """
    try:
        package_name, message_name = ros_message_type.split('/')
    except ValueError:
        raise ValueError(
            'ros_message_type should be in the shape of package_msgs/Message' +
            ' (it was ' + ros_message_type + ')')
    msg_class = locate('{}.srv.{}'.format(package_name, message_name))
    if msg_class is None:
        return False
    else:
        return True


def get_ROS_class(ros_message_type, srv=False):
    """
    Returns the ROS message class from ros_message_type.
    :return AnyMsgClass: Class of the ROS message.
    """
    try:
        package_name, message_name = ros_message_type.split('/')
    except ValueError:
        raise ValueError(
            'ros_message_type should be in the shape of package_msgs/Message' +
            ' (it was ' + ros_message_type + ')')
    if not srv:
        msg_class = locate('{}.msg.{}'.format(package_name, message_name))
    else:
        msg_class = locate('{}.srv.{}'.format(package_name, message_name))
    if msg_class is None:
        if srv:
            msg_or_srv = '.srv'
        else:
            msg_or_srv = '.msg'
        raise ValueError(
            'ros_message_type could not be imported. (' +
            ros_message_type + ', as "from ' + package_name +
            msg_or_srv + ' import ' + message_name + '" failed.')
    return msg_class


def from_ROS_to_dict(ros_msg):
    """
    Converts from a ROS message instance to a dict representation
    of it.
    :param AnyMsgInstance ros_msg: ROS message instance to transform to
        a dictionary.
    :returns dict: dictionary representing the ROS message.
    """
    # Note that this may be very slow for big data structures like
    # an sensor_msgs/Image
    return yaml.safe_load(ros_msg.__str__())


def from_dict_to_ROS(dict_msg, ros_message_type, srv=False):
    """
    Converts from a dict representation of a ROS message to a
    ROS message instance.
    """
    msg_class = get_ROS_class(ros_message_type, srv=srv)
    #print(msg_class)
    msg_instance = msg_class()
    #print(msg_instance.__slots__)
    #print(msg_instance._slot_types)
    #print(dict_msg.keys())
    if "data" in dict_msg.keys():
        import base64
        dict_msg["data"] = base64.b64decode(dict_msg["data"])
        print(dict_msg["data"][:10])
        #dict_msg["data"] = (dict_msg["data"]).encode()
        
    if "min_solution_cost" in dict_msg.keys():
        dict_msg["min_solution_cost"] = 0.0
    #for message_slot in msg_instance.__slots__:
    #    print(message_slot, dict_msg[message_slot])
    # Workaround
    if len(dict_msg) == 1:
        dict_msg = [dict_msg]
    fill_message_args(msg_instance, dict_msg)
    #print("instance: " + str(msg_instance) + str(len(msg_instance.data)))
    print(str(msg_instance)[:200])
    return msg_instance


def from_JSON_to_ROS(json_msg, ros_message_type, srv=False):
    """
    Converts from a dict representation of a ROS message to a
    ROS message instance.
    """
    return from_dict_to_ROS(from_JSON_to_dict(json_msg),
                            ros_message_type, srv=srv)


def from_JSON_to_dict(json_msg):
    """
    Converts from a json representation of a ROS message
    to a dict representation.
    """
    return json.loads(json_msg)


def from_dict_to_JSON(dict_msg):
    """
    Converts from a dict representation fo a ROS message
    to a JSON representation of it.
    """
    return json.dumps(dict_msg)


def from_ROS_to_JSON(ros_msg):
    """
    Converts from a ROS message instance to a JSON representation
    of it.
    """
    dict_msg = from_ROS_to_dict(ros_msg)
    return from_dict_to_JSON(dict_msg)


if __name__ == '__main__':
    from std_msgs.msg import Header
    h_ROS = Header()
    print("ROS message is:\n" + str(h_ROS))

    print("--------------------")
    print("From ROS message...")
    h_dict = from_ROS_to_dict(h_ROS)
    print("... to dict:\n" + str(h_dict))
    h_JSON = from_ROS_to_JSON(h_ROS)
    print("... to JSON:\n" + str(h_JSON))

    print("--------------------")
    print("From dict to...")
    h_ROS_from_dict = from_dict_to_ROS(h_dict, get_ROS_msg_type(h_ROS))
    print("... to ROS message:\n" + str(h_ROS_from_dict))
    h_JSON_from_dict = from_dict_to_JSON(h_dict)
    print("... to JSON:\n" + str(h_JSON_from_dict))

    print("--------------------")
    print("From JSON to...")
    h_ROS_from_JSON = from_JSON_to_ROS(h_JSON, get_ROS_msg_type(h_ROS))
    print("... to ROS message:\n" + str(h_ROS_from_JSON))
    h_dict_from_JSON = from_JSON_to_dict(h_JSON)
    print("... to dict:\n" + str(h_dict_from_JSON))

    print(("--------------------"))
    if h_ROS == h_ROS_from_dict:
        print("from_dict_to_ROS works.")
    else:
        print("from_dict_to_ROS error.")

    if h_ROS == h_ROS_from_JSON:
        print("from_JSON_to_ROS works.")
    else:
        print("from_JSON_to_ROS error.")

    if h_dict == h_dict_from_JSON:
        print("from_JSON_to_dict works.")
    else:
        print("from_JSON_to_dict error.")

    if h_JSON == h_JSON_from_dict:
        print("from_dict_to_JSON works.")
    else:
        print("from_dict_to_JSON error.")

    if h_dict == h_dict_from_JSON:
        print("from_ROS_to_dict works.")
    else:
        print("from_ROS_to_dict error.")

    if h_JSON == h_JSON_from_dict:
        print("from_ROS_to_JSON works.")
    else:
        print("from_ROS_to_JSON error.")
