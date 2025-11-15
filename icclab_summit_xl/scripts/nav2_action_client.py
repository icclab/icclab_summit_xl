import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys

# from nav2_action_client import ActionClient
from nav2_msgs.action import NavigateToPose


class Nav2ActionClient(Node):

    def __init__(self):
        super().__init__('nav2_action_client')
        self._action_client = ActionClient(self, NavigateToPose, '/summit/navigate_to_pose')
        self.info("nav2_action_client created")

    def send_goal(self, args):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(args[1])
        goal_msg.pose.pose.position.y = float(args[2])
        self.info("nav2_action_client waiting for server")
        if self._action_client.wait_for_server(timeout_sec=2):
            self.info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
                      str(goal_msg.pose.pose.position.y))
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback)
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()

            if not self.goal_handle.accepted:
                self.error('Goal to ' + str(goal_msg.pose.pose.position.x) + ' ' +
                            str(goal_msg.pose.pose.position.y) + ' was rejected!')
                return False

            goal_future = self.goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, goal_future)
            return True
        else:
            self.error("Timed out waiting for server")

        

    def nav_feedback(self, msg):
        self.info("Distance to goal: " + str(msg.feedback.distance_remaining))

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(args=None):  
    if args and len(args) == 3:    
        rclpy.init(args=args)
        action_client = Nav2ActionClient()
        action_client.send_goal(args)
    else:
        print("Usage: nav2_action_client.py x-coord y-coord")

if __name__ == '__main__':
    main(args=sys.argv)

