import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rclpy.clock import clock

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# TODO: UNCOMMENT BELOW LINES TO CONNECT AZENTA WHEN THEY ARE READY TO USE

# from azenta_driver.sealer_driver import A4S_SEALER_CLIENT  # import sealer driver

class SealerDescriptionClient(Node):

    def __init__(self, TEMP_NODE_NAME = 'SealerDescriptionNode'):
        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name() # Getting the real name from the launch file

        # TODO: UNCOMMENT BELOW LINES TO CONNECT AZENTA WHEN THEY ARE READY TO USE

        # self.declare_parameter('sealer_port', '/dev/ttyUSB1')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        # PORT = self.get_parameter('sealer_port') 

        # self.sealer = A4S_SEALER_CLIENT(PORT.value)

        timer_period = 0.1  # seconds

        self.state = "UNKNOWN"
        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, node_name + '/state',10)
        # self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)

        self.joint_publisher = self.create_publisher(JointState,'joint_states', 10, callback_group = joint_cb_group)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback, callback_group = joint_cb_group)
    
    
    def stateCallback(self):
        '''
        Publishes the azenta_description state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing State: "%s"' % msg.data)
        self.state = "READY"


    def joint_state_publisher_callback(self):
        # TODO: UNCOMMENT BELOW LINES TO CONNECT AZENTA WHEN THEY ARE READY TO USE

        # joint_states_sealer = self.peeler.refresh_joint_state()

        joint_states_sealer = [0.0]


        sealer_joint_msg = JointState()
        sealer_joint_msg.header = Header()
        sealer_joint_msg.header.stamp = self.get_clock().now().to_msg()
        sealer_joint_msg.name = ['Sealer_Plate_Joint']
        sealer_joint_msg.position = joint_states_sealer
        # print(joint_states)

        # azenta_joint_msg.position = [0.01, -1.34, 1.86, -3.03, 0.05, 0.05, 0.91]

        sealer_joint_msg.velocity = []
        sealer_joint_msg.effort = []

        self.joint_publisher.publish(sealer_joint_msg)
        self.get_logger().info('Publishing sealer joint states: "%s"' % str(joint_states_sealer))

def main(args=None):
    rclpy.init(args=args)
    try:
        sealer_joint_states_node = SealerDescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(sealer_joint_states_node)

        try:
            sealer_joint_states_node.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            sealer_joint_states_node.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            sealer_joint_states_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()