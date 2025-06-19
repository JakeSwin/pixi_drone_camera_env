import argparse
import os

import rclpy
import yaml
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node

# from your_package_name.msg import GeoPoint


def parse_arguments():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "-f",
        help="Map file to load. Must be .yaml filepath with 'field_boundary' list",
        dest="map_path",
        default="./src/maps/ri109.yaml",
        type=str,
    )
    args = parser.parse_args()
    return args


class FieldBoundaryPublisher(Node):
    def __init__(self, map_path: str):
        super().__init__("field_boundary_publisher")
        self.publisher_ = self.create_publisher(Polygon, "field_boundary", 10)
        self.timer = self.create_timer(
            1.0, self.publish_boundary
        )  # Publish every second
        map_path = os.path.abspath(map_path)
        self.boundary_points = self.load_boundary_from_yaml(map_path)

    def load_boundary_from_yaml(self, file_path):
        with open(file_path, "r") as file:
            data = yaml.safe_load(file)
            return data["field_boundary"]

    def publish_boundary(self):
        boundary = Polygon()
        for point in self.boundary_points:
            geo_point = Point32()
            geo_point.x = point["latitude"]
            geo_point.y = point["longitude"]
            boundary.points.append(geo_point)

        self.publisher_.publish(boundary)
        self.get_logger().info("Publishing field boundary data")


def main(args):
    rclpy.init(args=None)
    field_boundary_publisher = FieldBoundaryPublisher(args.map_path)
    rclpy.spin(field_boundary_publisher)
    field_boundary_publisher.destroy_node()
    rclpy.shutdown()


def main_and_args():
    args = parse_arguments()
    main(args)


if __name__ == "__main__":
    args = parse_arguments()
    main(args)
