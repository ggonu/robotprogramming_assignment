import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from multiply_interfaces.msg import Uid
from multiply_interfaces.srv import Multiply
from multiply_interfaces.action import AddDigits


class SubscriberClientAction(Node):
    def __init__(self):
        super().__init__('publisher_subscriber_action')
        self.subscription = self.create_subscription(Uid, 'Unversity_ID', self.listener_callback, 10)   # Customized interface를 사용해 uid subscribing
        self.cli = self.create_client(Multiply, 'multiply')     # Service Client
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable, waiting again...')
        self.req = Multiply.Request()
        self._action_client = ActionClient(self, AddDigits, 'add_digits')   # Action Client
        self.declare_parameter('geon_uid', '2020741056')    # Parameter
        
    def listener_callback(self, msg):
        self.get_logger().info('Recieved(UID) "%s"' % msg.uid)
        
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()
    
    def send_goal(self):
        geon_param = self.get_parameter('geon_uid').get_parameter_value().string_value  # Set Patameter
        
        new_geon_param = rclpy.parameter.Parameter(
            'geon_uid',
            rclpy.Parameter.Type.STRING,
            '2020741056'
        )
        all_new_params = [new_geon_param]
        self.set_parameters(all_new_params)
        
        goal_msg = AddDigits.Goal()     # Set action Goal
        goal_msg.uid = geon_param
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected...')
            return
        
        self.get_logger().info('Goal Accepted!')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: %d' % result.sequence[0])
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Recieved feedback: [First element is partial sum] {0}'.format(feedback.partial_sequence))
        
        
def main(args=None):
    rclpy.init(args=args)

    sub_cli_action = SubscriberClientAction()
    response = sub_cli_action.send_request(int(sys.argv[1]), int(sys.argv[2]))
    sub_cli_action.get_logger().info(
        'Result of multiply: %d * %d = %d'
        % (int(sys.argv[1]), int(sys.argv[2]), response.res)
    )
    sub_cli_action.send_goal()

    rclpy.spin(sub_cli_action)
    
    sub_cli_action.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    