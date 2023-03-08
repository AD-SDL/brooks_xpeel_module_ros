#! /usr/bin/env python3
"""Peeler node"""

from typing import List, Tuple
from time import sleep
import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String

from wei_services.srv import WeiActions, WeiDescription

from azenta_driver.peeler_driver import BROOKS_PEELER_DRIVER  # import peeler driver


class PeelerClient(Node):
    """
    The peelerNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback,
    based on the executed command and publishes the state of the peeler and a description of the peeler to the respective topics.
    """

    def __init__(self, TEMP_NODE_NAME = "PeelerNode"):
        """
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        """

        super().__init__(TEMP_NODE_NAME)
        self.node_name = self.get_name()


        self.declare_parameter('peeler_port', '/dev/ttyUSB1')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        self.PORT = self.get_parameter('peeler_port').get_parameter_value().string_value     # Renaming parameter to general form so it can be used for other nodes too
        self.get_logger().info("Received Port: " + str(self.PORT))

        self.state = 'UNKNOWN'
        self.robot_status = ""
        self.action_flag = "READY"
        self.reset_request_count = 0
        self.peeler = None
        self.connect_robot()

        self.description = {
            'name': self.node_name,
            'type':'',
            'actions':
            {
                'peel':'%d %d'
            }
            }

        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()
        state_refresher_cb_group = ReentrantCallbackGroup()

        timer_period = 1  # seconds
        state_refresher_timer_period = 1 # seconds
       
        self.StateRefresherTimer = self.create_timer(state_refresher_timer_period, callback = self.robot_state_refresher_callback, callback_group = state_refresher_cb_group)
       
        self.statePub = self.create_publisher(String, self.node_name + "/state", 10)       # Publisher for peeler state
        self.stateTimer = self.create_timer(timer_period, self.stateCallback, callback_group=state_cb_group)   # Callback that publishes to peeler state

        self.actionSrv = self.create_service(WeiActions, self.node_name + "/action_handler", self.actionCallback,callback_group=action_cb_group)
        self.descriptionSrv = self.create_service(WeiDescription, self.node_name + "/description_handler", self.descriptionCallback, callback_group=description_cb_group)
    
    def connect_robot(self):
        """Connect to robot"""

        try:
            self.peeler = BROOKS_PEELER_DRIVER(self.PORT)

        except Exception as err:
            self.state = "PEELER CONNECTION ERROR"
            self.get_logger().error("PEELER CONNECTION ERROR! ERROR: " + str(err))
        else: 
            self.get_logger().info("Peeler is online ")
            

    def robot_state_refresher_callback(self):
        
        try:
            if self.action_flag.upper() == "READY" and self.reset_request_count == 0: #Only refresh the state manualy if robot is not running a job.
                self.peeler.get_status()
 
            elif self.reset_request_count == 5: #Call the resetting function for one time at a time to ignore BUSY PORT erorrs. 
                self.peeler.reset() #Resets the robot state. 
                self.reset_request_count = 0

        except Exception as err:
            self.get_logger().error(str(err))

    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""
        msg = String()

        try:
            peeler_msg = self.peeler.status_msg
            # self.get_logger().info(peeler_msg)
        except Exception as err:
            self.get_logger().error("PEELER IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "PEELER CONNECTION ERROR"


        if self.state != "PEELER CONNECTION ERROR":
            
            if self.state == "ERROR" or "NO ERRORS" not in self.peeler.error_msg.upper() and self.peeler.error_msg != "":
                msg.data = 'State: ERROR'
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.get_logger().error(self.peeler.error_msg.upper())
                self.get_logger().warn('Trying to reset the Peeler')
                self.reset_request_count += 1
                self.action_flag = "READY"
                self.state = "UNKOWN"

            elif self.state == "COMPLETED" and self.action_flag == "BUSY":
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.action_flag = "READY"   

            elif self.peeler.movement_state == "BUSY" or self.action_flag == "BUSY":
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.peeler.status_msg == "READY" and self.peeler.movement_state == "READY" and self.action_flag == "READY":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)


        else:
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! PORT: " + str(self.PORT))
            self.action_flag = "READY"
            self.connect_robot()



    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response
        
        
    def actionCallback(self, request, response):
        """The actions the robot can perform, also performs them

        Parameters:
        -----------
        request: str
            Request to the robot to perform an action
        respone: bool
            If action is performed

        Returns
        -------
        None
        """
        if self.state == "PEELER CONNECTION ERROR":
            message = "Connection error, cannot accept a job!"
            self.get_logger().error(message)
            response.action_response = -1
            response.action_msg = message
            return response

        while self.state != "READY":
            self.get_logger().warn("Waiting for Peeler to switch READY state...")
            sleep(0.2)

        action_handle = request.action_handle  # Run commands if manager sends corresponding command
        vars = eval(request.vars)
        self.get_logger().info(str(vars))
        
        self.action_flag = "BUSY"

        if action_handle=="status":
            self.get_logger().info('Starting Action: ' + request.action_handle.upper())

            try:
                self.peeler.reset()
                self.peeler.check_version()
                self.peeler.get_status()  

            except Exception as err:
                response.action_response = -1
                response.action_msg = self.node_name + " Peeler getting status failed. Error: " + err
                self.state = "ERROR"

            else:    
                response.action_response = 0
                response.action_msg= self.node_name + " Peel getting status successfully completed"
                self.state = "COMPLETED"

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response


        elif action_handle=="peel":

            self.get_logger().info('Starting Action: ' + request.action_handle.upper())

            try:
                self.peeler.seal_check()
                self.peeler.peel(1, 2.5)        
                    
            except Exception as err:
                self.state = "ERROR"
                response.action_response = -1
                response.action_msg = str(self.node_name) + " Peel plate failed. Error: " + str(err)
                self.get_logger().error(response.action_msg)

            else:    
                self.state = "COMPLETED"
                response.action_response = 0
                response.action_msg= str(self.node_name) + " Peel plate successfully completed"
                self.get_logger().info(response.action_msg)

            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response


        else: 
            msg = "UNKOWN ACTION REQUEST! Available actions: seal"
            response.action_response = -1
            response.action_msg= msg
            self.get_logger().error('Error: ' + msg)
            self.state = "ERROR"
            return response
        

def main(args=None):  # noqa: D103

    rclpy.init(args=args)       # initialize Ros2 communication

    try:
        peeler_client = PeelerClient()
        executor = MultiThreadedExecutor()
        executor.add_node(peeler_client)

        try:
            peeler_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            peeler_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            peeler_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":

    main()
