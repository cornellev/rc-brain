import struct
from dataclasses import dataclass
from typing import Generator

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2, PointField


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


def get_unpack_format(field: PointField, big_endian: bool):
    endian_format = ">" if big_endian else "<"
    count_format = str(field.count)
    type_format = ""
    match field.datatype:
        case PointField.INT8:
            type_format = "b"
        case PointField.UINT8:
            type_format = "B"
        case PointField.INT16:
            type_format = "h"
        case PointField.UINT16:
            type_format = "H"
        case PointField.INT32:
            type_format = "i"
        case PointField.UINT32:
            type_format = "I"
        case PointField.FLOAT32:
            type_format = "f"
        case PointField.FLOAT64:
            type_format = "d"
    return f"{endian_format}{count_format}{type_format}"


def parse_points(cloud: PointCloud2) -> Generator[Point3D, None, None]:
    # TODO: avoid repetition (if we care)
    (x_field,) = [field for field in cloud.fields if field.name == "x"]
    (y_field,) = [field for field in cloud.fields if field.name == "y"]
    (z_field,) = [field for field in cloud.fields if field.name == "z"]
    x_format = get_unpack_format(x_field)
    y_format = get_unpack_format(y_field)
    z_format = get_unpack_format(z_field)

    for i in range(len(cloud.data) // cloud.point_step):
        point_offset = i * cloud.point_step
        x = struct.unpack_from(
            x_format, buffer=cloud.data, offset=point_offset + x_field.offset
        )
        y = struct.unpack_from(
            y_format, buffer=cloud.data, offset=point_offset + y_field.offset
        )
        z = struct.unpack_from(
            z_format, buffer=cloud.data, offset=point_offset + z_field.offset
        )
        yield Point3D(x, y, z)


class OccupancyTransformerNode(Node):
    def __init__(self):
        super().__init__("occupancy_transformer")
        self.subscription = self.create_subscription(
            PointCloud2, "point_cloud", self.pc_callback, QoSPresetProfiles.SENSOR_DATA
        )
        self.publisher = self.create_publisher(OccupancyGrid, "occupancy_grid", 10)

        self.declare_parameter("map_height_m", 10.0)
        self.declare_parameter("map_width_m", 10.0)
        self.declare_parameter("map_resolution_m", 0.1)

    def pc_callback(self, cloud: PointCloud2):
        map_height_m = self.get_parameter("map_height_m").value
        map_width_m = self.get_parameter("map_width_m").value
        map_resolution_m = self.get_parameter("map_resolution_m").value

        map_height_cells = int(map_height_m / map_resolution_m)
        map_width_cells = int(map_width_m / map_resolution_m)


def main():
    rclpy.init()

    occupancy_transformer = OccupancyTransformerNode()
    rclpy.spin(occupancy_transformer)

    occupancy_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
