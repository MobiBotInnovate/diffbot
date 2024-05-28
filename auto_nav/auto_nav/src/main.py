import rclpy
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class AutoNav(Node):
    """
    Initiate all the nodes for subscription
    """

    def __init__(self):
        super().__init__("AutoNav")

        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.navigator = BasicNavigator()
        self.costmap = OccupancyGrid()
        # Wait for nav2 stack to start
        # self.navigator.waitUntilNav2Active(navigator="bt_navigator", localizer="none")

        # TODO: Make the program run continuosl
        # Set our demo's initial pose
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = "map"
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = -2.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 0.0
        # self.navigator.setInitialPose(initial_pose)
        #
        self.timer = self.create_timer(
            5, self.navigate_to_unexplored
        )  # Every 5 seconds

    def map_callback(self, msg):
        self.get_logger().info("Receieved new map")
        self.costmap = msg
        self.costmap_header = PyCostmap2D(msg)
        self.get_logger().info(f"Map data: {self.costmap_header}")

        self.navigate_to_unexplored()

    def find_unexplored_points(self):
        unexplored = []
        width = self.costmap_header.getSizeInCellsX()
        height = self.costmap_header.getSizeInCellsY()
        resolution = self.costmap_header.getResolution()
        originX = self.costmap_header.getOriginX()
        originY = self.costmap_header.getOriginY()
        data = self.costmap_data.data

        for y in range(height):
            for x in range(width):
                index = x + y * width
                if data[index] == -1:  # unknown
                    worldX = originX + (x * 0.5) * resolution
                    worldY = originY + (y * 0.5) * resolution
                    unexplored.append((worldX, worldY))
        return unexplored

    def navigate_to_unexplored(self):
        self.costmap_data = self.navigator.getGlobalCostmap()
        self.get_logger().info(f"Costmap data: {self.costmap_data}")
        unexplored_points = self.find_unexplored_points()

        for x, y in unexplored_points:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.z = 1
            goal_pose.pose.orientation.w = 0

        self.get_logger().info(f"Setting navigation goal to ({x}, {y})")
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self)
        if self.navigator.getResult().SUCCEEDED:
            self.get_logger().info("Reached the goal successfully")
        else:
            self.get_logger().info("Failed to reach the goal")


def main(args=None):
    rclpy.init(args=args)
    autonav = AutoNav()
    executor = MultiThreadedExecutor()
    try:
        executor.add_node(autonav)
        executor.spin()
    except KeyboardInterrupt:
        autonav.get_logger().info("AutoNav Node interrupted by user.")
    finally:
        autonav.navigator.lifecycleShutdown()
        autonav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
