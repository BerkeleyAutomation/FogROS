#!/usr/bin/env python3

import rospy
from .conversions import from_dict_to_JSON
from .conversions import from_JSON_to_dict
from .conversions import from_dict_to_ROS
from .conversions import from_ROS_to_dict
from .conversions import from_JSON_to_ROS
from .conversions import from_ROS_to_JSON
from .conversions import get_ROS_msg_type
from .conversions import get_ROS_class
from .conversions import is_ros_message_installed, is_ros_service_installed
from pydoc import locate
import socket

from .rosbridge_client import ROSBridgeClient

"""
Server to expose locally and externally
topics, services and parameters from a remote
roscore to a local roscore.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

yaml_config = '''
# ROSbridge websocket server info
rosbridge_ip: 192.168.1.31
rosbridge_port: 9090
# Topics being published in the robot to expose locally
remote_topics: [
                    ['/joint_states', 'sensor_msgs/JointState'], 
                    ['/tf', 'tf2_msgs/TFMessage'],
                    ['/scan', 'sensor_msgs/LaserScan']
                    ]
# Topics being published in the local roscore to expose remotely
local_topics: [
                    ['/test1', 'std_msgs/String'],
                    ['/closest_point', 'sensor_msgs/LaserScan']
                    ]
# Services running in the robot to expose locally
remote_services: [
                    ['/rosout/get_loggers', 'roscpp/GetLoggers']
                    ]
# Services running locally to expose to the robot
local_services: [
                    ['/add_two_ints', 'beginner_tutorials/AddTwoInts']
                    ]
# Parameters to be sync, they will be polled to stay in sync
parameters: ['/robot_description']
parameter_polling_hz: 1'''


class ROSduct(object):
    def __init__(self):
        # ROSbridge
        self.rosbridge_ip = rospy.get_param('~rosbridge_ip', None)
        if self.rosbridge_ip is None:
            rospy.logerr('No rosbridge_ip given.')
            raise Exception('No rosbridge_ip given.')
        self.rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
        rospy.loginfo("Will connect to ROSBridge websocket: ws://{}:{}".format(
            self.rosbridge_ip, self.rosbridge_port))

        # Topics
        # TODO: check if topic types are installed, if not, give a warning
        self.remote_topics = rospy.get_param('~remote_topics', [])
        rospy.loginfo("Remote topics: " + str(self.remote_topics))
        self.local_topics = rospy.get_param('~local_topics', [])
        rospy.loginfo("Local topics: " + str(self.local_topics))

        # Services
        # TODO: check if service types are installed
        self.remote_services = rospy.get_param('~remote_services', [])
        rospy.loginfo("Remote services: " + str(self.remote_services))
        self.local_services = rospy.get_param('~local_services', [])
        rospy.loginfo("Local services: " + str(self.local_services))

        # Parameters
        self.rate_hz = rospy.get_param('~parameter_polling_hz', 1)
        self.parameters = rospy.get_param('~parameters', [])
        rospy.loginfo("Parameters: " + str(self.parameters))
        self.last_params = {}

        if rospy.get_param("~automatic_topic_scan", "True") == "True":
            self.automatic_scanning = True
        else:
            self.automatic_scanning = False

        self.check_if_msgs_are_installed()

        self.initialize()

    def new_remote_topics(self):
        for r_t in self.remote_topics:
            self.new_remote_topic(r_t)

    def new_remote_topic(self, r_t):
        if 1:
            if len(r_t) == 2:
                topic_name, topic_type = r_t
                local_name = topic_name
            elif len(r_t) == 3:
                topic_name, topic_type, local_name = r_t
            rospub = rospy.Publisher(local_name,
                                     get_ROS_class(topic_type),
                                     # SubscribeListener added later                          
                                     queue_size=1)

            cb_r_to_l = self.create_callback_from_remote_to_local(topic_name,
                                                                  topic_type,
                                                                  rospub)
            subl = self.create_subscribe_listener(topic_name,
                                                  topic_type,
                                                  cb_r_to_l)
            rospub.impl.add_subscriber_listener(subl)
            self._instances['topics'].append(
                {topic_name:
                 {'rospub': rospub,
                  'bridgesub': None}
	         })

    def new_local_topics(self):
        for l_t in self.local_topics:
            self.new_local_topic(l_t)

    def new_local_topic(self, l_t):
        if 1:
            if len(l_t) == 2:
                topic_name, topic_type = l_t
                remote_name = topic_name
            elif len(l_t) == 3:
                topic_name, topic_type, remote_name = l_t

            bridgepub = self.client.publisher(remote_name, topic_type)

            cb_l_to_r = self.create_callback_from_local_to_remote(topic_name,
                                                                  topic_type,
                                                                  bridgepub)

            rossub = rospy.Subscriber(topic_name,
                                      get_ROS_class(topic_type),
                                      cb_l_to_r)
            self._instances['topics'].append(
                {topic_name:
                 {'rossub': rossub,
                  'bridgepub': bridgepub}
                 })
    
    def new_remote_services(self):
        for r_s in self.remote_services:
            self.new_remote_service(r_s)
            
    def new_remote_service(self, r_s):
        if 1:
            if len(r_s) == 2:
                service_name, service_type = r_s
                local_name = service_name
            elif len(r_s) == 3:
                service_name, service_type, local_name = r_s
            remote_service_client = self.client.service_client(service_name,
                                                               service_type)
            r_to_l_serv_cb = self.create_callback_from_remote_to_local_srv(
                remote_service_client,
                service_name,
                service_type)
            rosserv = rospy.Service(local_name,
		                    get_ROS_class(service_type,
                                                  srv=True),
                                    r_to_l_serv_cb)

            self._instances['services'].append(
                {service_name:
                 {'rosserv': rosserv,
                  'bridgeservclient': remote_service_client}
		 })

    def new_local_services(self):
        for l_s in self.local_services:
            self.new_local_service(l_s)
            
    def new_local_service(self, l_s):
        if 1:
            if len(l_s) == 2:
                service_name, service_type = l_s
                remote_name = service_name
            elif len(l_s) == 3:
                service_name, service_type, remote_name = l_s
            rosservprox = rospy.ServiceProxy(service_name,
                                             get_ROS_class(service_type,
                                                           srv=True))
            l_to_r_srv_cv = self.create_callback_from_local_to_remote_srv(
                service_name,
                service_type,
                rosservprox)
            remote_service_server = self.client.service_server(remote_name,
                                                               service_type,
                                                               l_to_r_srv_cv)
            self._instances['services'].append(
                {service_name:
                 {'rosservprox': rosservprox,
                  'bridgeservserver': remote_service_server}
                 })

    def initialize(self):
        """
        Initialize creating all necessary bridged clients and servers.
        """
        connected = False
        while not rospy.is_shutdown() and not connected:
            try:
                self.client = ROSBridgeClient(
                    self.rosbridge_ip, self.rosbridge_port)
                connected = True
            except socket.error as e:
                rospy.logwarn(
                    'Error when opening websocket, is ROSBridge running?')
                rospy.logwarn(e)
                rospy.sleep(5.0)

        # We keep track of the instanced stuff in this dict
        self._instances = {'topics': [],
                           'services': []}

        self.new_remote_topics()
        self.new_local_topics()
        self.new_remote_services()
        self.new_local_services()
        
        # Get all params and store them for further updates
        for param in self.parameters:
            if type(param) == list:
                # remote param name is the first one
                param = param[0]
            self.last_params[param] = self.client.get_param(param)

    def create_callback_from_remote_to_local(self, topic_name,
                                             topic_type,
                                             rospub):
        # Note: argument MUST be named 'message' as
        # that's the keyword given to pydispatch
        def callback_remote_to_local(message):
            print("Remote ROSBridge subscriber from topic " +
                           topic_name + ' of type ' +
                           topic_type + ' got data: ' + 
                           ' which is republished locally.')
            # Only convert and publish with subscribers
            if rospub.get_num_connections() >= 1:
                msg = from_dict_to_ROS(message, topic_type)
                #try:
                rospub.publish(msg)
                #except Exception as e:
                #    print("published" + str(e))
        return callback_remote_to_local

    def create_callback_from_local_to_remote(self,
                                             topic_name,
                                             topic_type,
                                             bridgepub):
        def callback_local_to_remote(message):
            rospy.logdebug("Local subscriber from topic " +
                           topic_name + ' of type ' +
                           topic_type + ' got data: ' + str(message) +
                           ' which is republished remotely.')
            dict_msg = from_ROS_to_dict(message)
            bridgepub.publish(dict_msg)
        return callback_local_to_remote

    def create_subscribe_listener(self,
                                  topic_name,
                                  topic_type,
                                  cb_r_to_l):
        # We create a SubscribeListener that will
        # create a rosbridge subscriber on demand
        # and also unregister it if no one is listening
        class CustomSubscribeListener(rospy.SubscribeListener):
            def __init__(this):
                super(CustomSubscribeListener, this).__init__()
                this.bridgesub = None

            def peer_subscribe(this, tn, tp, pp):
                # Only make a new subscriber if there wasn't one
                if this.bridgesub is None:
                    rospy.logdebug(
                        "We have a first subscriber to: " + topic_name)
                    this.bridgesub = self.client.subscriber(
                        topic_name,
                        topic_type,
                        cb_r_to_l)
                    for idx, topic_d in enumerate(self._instances['topics']):
                        if topic_d.get(topic_name):
                            self._instances['topics'][idx][topic_name]['bridgesub'] = this.bridgesub
                            break

            def peer_unsubscribe(this, tn, num_peers):
                # Unsubscribe if there isnt anyone left
                if num_peers < 1:
                    rospy.logdebug(
                        "There are no more subscribers to: " + topic_name)
                    self.client.unsubscribe(this.bridgesub)
                    this.bridgesub = None
                    # May be redundant if it's actually a reference to this.bridgesub already
                    for idx, topic_d in enumerate(self._instances['topics']):
                        if topic_d.get(topic_name):
                            self._instances['topics'][idx][topic_name]['bridgesub'] = None
                            break
        return CustomSubscribeListener()

    def create_callback_from_remote_to_local_srv(self,
                                                 remote_service_client,
                                                 service_name,
                                                 service_type):
        def callback_from_local_srv_call(request):
            rospy.loginfo("Got a SRV call to " + service_name +
                          " of type " + service_type)
            req_dict = from_ROS_to_dict(request)

            result = {
                'responded': False,
                'response': None
            }

            def cb(success, response):
                result['responded'] = True
                if success:
                    result['response'] = response
            remote_service_client.request(req_dict, cb)
            while not rospy.is_shutdown() and not result['responded']:
                rospy.sleep(0.1)
            if result['response'] is None:
                rospy.logerr('Service call didnt succeed (' + str(service_name) +
                             ' of type ' + str(service_type))
                return None
            return from_dict_to_ROS(result['response'],
                                    service_type + 'Response',
                                    srv=True)
        return callback_from_local_srv_call

    def create_callback_from_local_to_remote_srv(self,
                                                 service_name,
                                                 service_type,
                                                 rosservprox):

        def callback_from_remote_service_call(request):
            ros_req = from_dict_to_ROS(request, service_type + 'Request',
                                       srv=True)
            rospy.loginfo("Waiting for server " + service_name + "...")
            rospy.wait_for_service(service_name)
            # TODO: error handling in services...
            resp = rosservprox.call(ros_req)
            resp_dict = from_ROS_to_dict(resp)
            return True, resp_dict

        return callback_from_remote_service_call

    def check_if_msgs_are_installed(self):
        """
        Check if the provided message types are installed.
        """
        for rt in self.remote_topics:
            if len(rt) == 2:
                _, topic_type = rt
            elif len(rt) == 3:
                _, topic_type, _ = rt
            if not is_ros_message_installed(topic_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(topic_type))

        for lt in self.local_topics:
            if len(lt) == 2:
                _, topic_type = lt
            elif len(lt) == 3:
                _, topic_type, _ = lt
            if not is_ros_message_installed(topic_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(topic_type))

        for rs in self.remote_services:
            if len(rs) == 2:
                _, service_type = rs
            elif len(rs) == 3:
                _, service_type, _ = rs
            if not is_ros_service_installed(service_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(service_type))

        for ls in self.local_services:
            if len(ls) == 2:
                _, service_type = ls
            elif len(ls) == 3:
                _, service_type, _ = ls
            if not is_ros_service_installed(service_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(service_type))

    def sync_params(self):
        """
        Sync parameter server in between
        external and local roscore (local changes
        are not forwarded).
        """
        for param in self.parameters:
            if type(param) == list:
                local_param = param[1]
                param = param[0]
            else:
                local_param = param
            # Get remote param
            remote_param = self.client.get_param(param)
            if remote_param != self.last_params[param]:
                rospy.set_param(local_param, remote_param)
                self.last_params[param] = remote_param

    def sync_topics(self):
        current_remote_topics = self.client.get_topics()
        current_local_topic_with_types = rospy.get_published_topics()
        current_local_topics = [topic[0] for topic in current_local_topic_with_types]
        add_to_local_topics = set(current_remote_topics) - set(current_local_topics)
        add_to_remote_topics = set(current_local_topics) - set(current_remote_topics)
        print("Add to local topics", add_to_local_topics)
        print("Add to remote topics", add_to_remote_topics)

        # add to local from remote topic
        for r_t in add_to_local_topics:
            topic_type = self.client.get_topic_type(r_t)
            self.new_remote_topic([r_t, topic_type])

        # add to remote from local topic
        for l_t in add_to_remote_topics:
            for n, t in current_local_topic_with_types:
                if n == l_t:
                    topic_type = t
            self.new_local_topic([l_t, topic_type])

    
    def sync_topics(self):
        current_remote_topics = self.client.get_topics()
        current_local_topic_with_types = rospy.get_published_topics()
        current_local_topics = [topic[0] for topic in current_local_topic_with_types]
        add_to_local_topics = set(current_remote_topics) - set(current_local_topics)
        add_to_remote_topics = set(current_local_topics) - set(current_remote_topics)
        #print("Add to local topics", add_to_local_topics)
        #print("Add to remote topics", add_to_remote_topics)

        # add to local from remote topic
        for r_t in add_to_local_topics:
            topic_type = self.client.get_topic_type(r_t)
            self.new_remote_topic([r_t, topic_type])

        # add to remote from local topic
        for l_t in add_to_remote_topics:
            for n, t in current_local_topic_with_types:
                if n == l_t:
                    topic_type = t
            self.new_local_topic([l_t, topic_type])
    
    def sync_topics(self):
        current_remote_topics = self.client.get_topics()
        current_local_topic_with_types = rospy.get_published_topics()
        current_local_topics = [topic[0] for topic in current_local_topic_with_types]
        add_to_local_topics = set(current_remote_topics) - set(current_local_topics)
        add_to_remote_topics = set(current_local_topics) - set(current_remote_topics)
        #print("Add to local topics", add_to_local_topics)
        #print("Add to remote topics", add_to_remote_topics)

        # add to local from remote topic
        for r_t in add_to_local_topics:
            topic_type = self.client.get_topic_type(r_t)
            self.new_remote_topic([r_t, topic_type])

        # add to remote from local topic
        for l_t in add_to_remote_topics:
            for n, t in current_local_topic_with_types:
                if n == l_t:
                    topic_type = t
            self.new_local_topic([l_t, topic_type])

    def sync_topics(self):
        current_remote_topics = self.client.get_topics()
        current_local_topic_with_types = rospy.get_published_topics()
        current_local_topics = [topic[0] for topic in current_local_topic_with_types]
        add_to_local_topics = set(current_remote_topics) - set(current_local_topics)
        add_to_remote_topics = set(current_local_topics) - set(current_remote_topics)
        #print("Add to local topics", add_to_local_topics)
        #print("Add to remote topics", add_to_remote_topics)

        # add to local from remote topic
        for r_t in add_to_local_topics:
            topic_type = self.client.get_topic_type(r_t)
            self.new_remote_topic([r_t, topic_type])

        # add to remote from local topic
        for l_t in add_to_remote_topics:
            for n, t in current_local_topic_with_types:
                if n == l_t:
                    topic_type = t
            self.new_local_topic([l_t, topic_type])

    def sync_services(self):
        import rosservice
        current_remote_services = [service for service in self.client.get_services() if not service.startswith("/ros")]
        current_local_services = [service for service in rosservice.get_service_list() if not service.startswith("/ros")]
        #print(current_remote_services)
        #print(current_local_services)
        add_to_local_services = set(current_remote_services) - set(current_local_services)
        add_to_remote_services = set(current_local_services) - set(current_remote_services)
        #print("Add to local services", add_to_local_services)
        #print("Add to remote services", add_to_remote_services)

        # add to local from remote service
        for r_s in add_to_local_services:
            service_type = self.client.get_service_type(r_s)
            self.new_remote_service([r_s, service_type])

        # add to remote from local service
        for l_s in add_to_remote_services:
            #for n, t in current_local_service_with_types:
            #    if n == l_t:
            #        service_type = t
            service_type = rosservice.get_service_type(l_s)
            self.new_local_service([l_s, service_type])

        
    def spin(self):
        """
        Run the node, needed to update the parameter server.
        """
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.sync_params()
            if self.automatic_scanning:
                self.sync_topics()
                self.sync_services()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('rosduct')
    r = ROSduct()
    r.spin()
