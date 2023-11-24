import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node

class Nav2GoalActionClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate2pose')
        self._goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self._send_goal_cb, 1)

    def _send_goal_cb(self, msg):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        self._send_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self._feedback_callback)

       

        self._send_future.add_done_callback(self._goal_response_callback)
        
        
    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self._get_result_callback)
        
    def _get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result))

            
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance remaining = {feedback.distance_remaining}")



def main(args=None):
    rclpy.init(args=args)

    action_client = Nav2GoalActionClient()

    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()