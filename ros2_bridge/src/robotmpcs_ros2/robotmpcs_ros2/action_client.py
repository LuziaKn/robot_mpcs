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
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        future.feedback_callback = self._feedback_callback
        #future.result_callback = self._result_callback
        
        # Wait for the result
        rclpy.spin_until_future_complete(self, future)
        

        result = future.result()
        self._action_client.wait_for_server()
        
        

            
    def _feedback_callback(self, feedback_msg):
        self.node.get_logger().info(f"Distance remaining = {feedback_msg.distance_remaining}")

    def _result_callback(self, future):
        result = future.result()
        if result.code == rclpy.action.ResultCode.STATUS_SUCCEEDED:
            self.node.get_logger().info("Success!!!")
        elif result.code == rclpy.action.ResultCode.STATUS_ABORTED:
            self.node.get_logger().error("Goal was aborted")
        elif result.code == rclpy.action.ResultCode.STATUS_CANCELED:
            self.node.get_logger().error("Goal was canceled")
        else:
            self.node.get_logger().error("Unknown result code")



def main(args=None):
    rclpy.init(args=args)

    action_client = Nav2GoalActionClient()

    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()