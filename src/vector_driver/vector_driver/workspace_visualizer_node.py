import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class WorkspaceVisualizer(Node):
    def __init__(self, marker_world):
        super().__init__('workspace_visualizer')
        self.pub = self.create_publisher(Marker, '/workspace_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)  # 1 Hz
        self.marker_world = marker_world

        # Precompute workspace rectangle in meters
        self.corners = [
            (-0.5, 0.5, 0.0),
            (0.5, 0.5, 0.0),
            (0.5, -0.5, 0.0),
            (-0.5, -0.5, 0.0),
            (-0.5, 0.5, 0.0)
        ]

    def publish_markers(self):
        # --- Rectangle border ---
        rect_marker = Marker()
        rect_marker.header.frame_id = "map"
        rect_marker.header.stamp = self.get_clock().now().to_msg()
        rect_marker.ns = "workspace_border"
        rect_marker.id = 0
        rect_marker.type = Marker.LINE_STRIP
        rect_marker.action = Marker.ADD
        rect_marker.scale.x = 0.01  # line width
        rect_marker.color.r = 1.0
        rect_marker.color.g = 0.0
        rect_marker.color.b = 0.0
        rect_marker.color.a = 1.0
        for x, y, z in self.corners:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            rect_marker.points.append(p)
        self.pub.publish(rect_marker)

        # --- Cubes for each marker ---
        for idx, m in enumerate(self.marker_world.marker_transforms.values(), start=1):
            cube_marker = Marker()
            cube_marker.header.frame_id = "map"
            cube_marker.header.stamp = self.get_clock().now().to_msg()
            cube_marker.ns = "markers"
            cube_marker.id = idx
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.pose.position.x = m["pos"][0] / 1000.0
            cube_marker.pose.position.y = m["pos"][1] / 1000.0
            cube_marker.pose.position.z = m["pos"][2] / 1000.0 + 0.02  # slightly above ground
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.04
            cube_marker.scale.y = 0.04
            cube_marker.scale.z = 0.04
            cube_marker.color.r = 0.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0
            cube_marker.color.a = 1.0
            self.pub.publish(cube_marker)


def main(args=None):
    rclpy.init(args=args)
    from vector_driver.world import Marker_World
    node = WorkspaceVisualizer(Marker_World())
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
