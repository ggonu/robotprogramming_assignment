import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from multiply_interfaces.msg import Uid
from multiply_interfaces.srv import Multiply
from multiply_interfaces.action import AddDigits


class PublisherServiceAction(Node):
    def __init__(self):
        super().__init__('publisher_service_action')
        self.publisher_ = self.create_publisher(Uid, 'University_ID', 10)
        self.srv = self.create_service(Multiply, 'multiply', self.multiply_callback)
        self.action_server = ActionServer(self, AddDigits, 'add_digits', self.exec_callback)
        self.declare_parameter('geon_uid', '2020741056')
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        geon_param = self.get_parameter('geon_uid').get_parameter_value().string_value
        
        new_geon_param = rclpy.parameter.Parameter(
            'geon_uid',
            rclpy.Parameter.Type.STRING,
            '2020741056'
        )
        all_new_params = [new_geon_param]
        self.set_parameters(all_new_params)
        
        msg = Uid()
        msg.uid = geon_param + ': %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing(UID): "%s"' % msg.uid)
        self.i += 1
        
    def multiply_callback(self, request, response):
        response.res = request.a * request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        
        return response
    
    def exec_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = AddDigits.Feedback()
        Uid = goal_handle.request.uid
        feedback_msg.partial_sequence = [int(digit) for digit in Uid]
        
        temp_sum = 0
        for digit in feedback_msg.partial_sequence:
            temp_sum += digit
            feedback_msg.partial_sequence[0] = temp_sum
            self.get_logger().info('Feedback: [First element is partial sum] {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
            
        goal_handle.succeed()
        
        result = AddDigits.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    
    pub_srv_action = PublisherServiceAction()
    rclpy.spin(pub_srv_action)
    
    pub_srv_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
