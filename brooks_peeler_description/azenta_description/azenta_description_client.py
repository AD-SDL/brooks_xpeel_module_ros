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

# from azenta_driver.peeler_driver import BROOKS_PEELER_CLIENT  # import peeler driver
# from azenta_driver.sealer_driver import A4S_SEALER_CLIENT  # import sealer driver

class AzentaDescriptionClient(Node):

    def __init__(self, NODE_NAME = 'AzentaDescriptionNode'):
        super().__init__(NODE_NAME)

        # TODO: UNCOMMENT BELOW LINES TO CONNECT AZENTA WHEN THEY ARE READY TO USE

        # self.declare_parameter('sealer_port', '/dev/ttyUSB1')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        # PORT = self.get_parameter('sealer_port') 

        # self.declare_parameter('peeler_port', '/dev/ttyUSB0')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        # PORT = self.get_parameter('peeler_port')    # Renaming parameter to general form so it can be used for other nodes too

        # self.peeler = BROOKS_PEELER_CLIENT(PORT.value)
        # self.sealer = A4S_SEALER_CLIENT(PORT.value)

        timer_period = 0.1  # seconds

        self.state = "UNKNOWN"
        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, NODE_NAME + '/state',10)
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

        # joint_states_peeler = self.peeler.refresh_joint_state()
        # joint_states_sealer = self.peeler.refresh_joint_state()

        joint_states_peeler = [0.0]
        joint_states_sealer = [0.0]

        peeler_joint_msg = JointState()
        peeler_joint_msg.header = Header()
        peeler_joint_msg.header.stamp = self.get_clock().now().to_msg()
        peeler_joint_msg.name = ['Peeler_Plate_Joint']
        peeler_joint_msg.position = joint_states_peeler

        sealer_joint_msg = JointState()
        sealer_joint_msg.header = Header()
        sealer_joint_msg.header.stamp = self.get_clock().now().to_msg()
        sealer_joint_msg.name = ['Sealer_Plate_Joint']
        sealer_joint_msg.position = joint_states_sealer
        # print(joint_states)

        # azenta_joint_msg.position = [0.01, -1.34, 1.86, -3.03, 0.05, 0.05, 0.91]
        peeler_joint_msg.velocity = []
        peeler_joint_msg.effort = []
        sealer_joint_msg.velocity = []
        sealer_joint_msg.effort = []

        self.joint_publisher.publish(peeler_joint_msg)
        self.get_logger().info('Publishing peeler joint states: "%s"' % str(joint_states_peeler))
        self.joint_publisher.publish(sealer_joint_msg)
        self.get_logger().info('Publishing sealer joint states: "%s"' % str(joint_states_sealer))

def main(args=None):
    rclpy.init(args=args)
    try:
        azenta_joint_state_publisher = AzentaDescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(azenta_joint_state_publisher)

        try:
            azenta_joint_state_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            azenta_joint_state_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            azenta_joint_state_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()