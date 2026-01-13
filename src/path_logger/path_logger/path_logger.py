import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
import os
from datetime import datetime
import math

class PathLogger(Node):

    def __init__(self):
        super().__init__('path_logger')
        self.sub = self.create_subscription(Path, '/plan', self.callback, 10)

        self.save_dir = os.path.expanduser('~/go2_solorzano/waypoints')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info("Path Logger activo. Guardando waypoints en ~/go2_solorzano/waypoints")

    def callback(self, msg):
        if len(msg.poses) == 0:
            return

        filename = datetime.now().strftime("path_%Y%m%d_%H%M%S.csv")
        filepath = os.path.join(self.save_dir, filename)

        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x','y','yaw'])

            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y

                q = pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z))

                writer.writerow([x,y,yaw])

        self.get_logger().info(f"Waypoints guardados en {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = PathLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
