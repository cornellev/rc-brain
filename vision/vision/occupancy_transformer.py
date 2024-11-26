import struct

import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField


class OccupancyTransformerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("occupancy_transformer")

        self.declare_parameter("point_cloud_topic", "point_cloud")
        self.declare_parameter("map_height_m", 10.0)
        self.declare_parameter("map_width_m", 10.0)
        self.declare_parameter("map_resolution_m", 0.1)
        self.declare_parameter("ride_height_m")
        self.declare_parameter("car_height_m")
        self.declare_parameter("fuzz_threshold_m")
        self.declare_parameter("base_link_frame", "base_link")

        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("point_cloud_topic").value,
            self.pc_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "occupancy", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def pc_callback(self, cloud: PointCloud2):
        map_height_m = self.get_parameter("map_height_m").value
        map_width_m = self.get_parameter("map_width_m").value
        map_resolution_m = self.get_parameter("map_resolution_m").value

        map_height_cells = int(map_height_m / map_resolution_m)
        map_width_cells = int(map_width_m / map_resolution_m)

        ride_height_m = self.get_parameter("ride_height_m").value
        car_height_m = self.get_parameter("car_height_m").value
        fuzz_threshold_m = self.get_parameter("fuzz_threshold_m").value

        base_link_frame = self.get_parameter("base_link_frame").value
        transform = self.tf_buffer.lookup_transform(
            base_link_frame, cloud.header.frame_id, rclpy.time.Time()
        )
        translation = transform.transform.translation
        translation = np.array([translation.x, translation.y, translation.z])
        rotation = transform.transform.rotation
        rotation = quaternion_to_rotation_matrix(
            (rotation.w, rotation.x, rotation.y, rotation.z)
        )

        point_cloud = parse_points(cloud)
        base_link_pc = point_cloud @ rotation.T + translation

        pc_z = base_link_pc[:, 2]
        filtered_pc = base_link_pc[
            (pc_z < car_height_m + fuzz_threshold_m)
            & (pc_z > ride_height_m - fuzz_threshold_m)
        ]

        grid = np.zeros((map_height_cells, map_width_cells), dtype=np.int8)

        cell_x = filtered_pc[:, 0] / map_resolution_m
        cell_y = (filtered_pc[:, 1] + map_width_m / 2) / map_resolution_m
        cell_pc = np.stack((cell_x, cell_y), axis=-1).astype(np.int32)

        in_range = cell_pc[
            (
                (cell_pc[:, 0] >= 0)
                & (cell_pc[:, 0] < map_height_cells)
                & (cell_pc[:, 1] >= 0)
                & (cell_pc[:, 1] < map_width_cells)
            )
        ]

        grid[in_range] = 1
        flat = grid.flatten()

        occupancy_grid = OccupancyGrid()

        occupancy_grid.header.stamp = cloud.header.stamp
        occupancy_grid.header.frame_id = base_link_frame

        occupancy_grid.info.map_load_time = cloud.header.stamp
        occupancy_grid.info.resolution = map_resolution_m
        occupancy_grid.info.width = map_width_cells
        occupancy_grid.info.height = map_height_cells

        origin_offset = (-map_width_cells * map_resolution_m) / 2
        map_origin = Pose(
            position=Point(x=0.0, y=origin_offset, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        occupancy_grid.info.origin = map_origin

        occupancy_grid.data = list(map(int, flat))

        self.publisher.publish(occupancy_grid)


def parse_points(cloud: PointCloud2) -> npt.NDArray[np.float32]:
    # TODO: avoid repetition (if we care)
    (x_field,) = [field for field in cloud.fields if field.name == "x"]
    (y_field,) = [field for field in cloud.fields if field.name == "y"]
    (z_field,) = [field for field in cloud.fields if field.name == "z"]
    x_format = get_unpack_format(x_field, cloud.is_bigendian)
    y_format = get_unpack_format(y_field, cloud.is_bigendian)
    z_format = get_unpack_format(z_field, cloud.is_bigendian)

    n_points = len(cloud.data) // cloud.point_step
    result = np.zeros((n_points, 3), dtype=np.float32)

    # TODO: this probably needs to get faster using iter_unpack and pad bytes
    for i in range(n_points):
        point_offset = i * cloud.point_step
        (result[i][0],) = struct.unpack_from(
            x_format, buffer=cloud.data, offset=point_offset + x_field.offset
        )
        (result[i][1],) = struct.unpack_from(
            y_format, buffer=cloud.data, offset=point_offset + y_field.offset
        )
        (result[i][2],) = struct.unpack_from(
            z_format, buffer=cloud.data, offset=point_offset + z_field.offset
        )

    return result


def get_unpack_format(field: PointField, big_endian: bool):
    endian_format = ">" if big_endian else "<"
    count_format = str(field.count)

    type_format_dict = {
        PointField.INT8: "b",
        PointField.UINT8: "B",
        PointField.INT16: "h",
        PointField.UINT16: "H",
        PointField.INT32: "i",
        PointField.UINT32: "I",
        PointField.FLOAT32: "f",
        PointField.FLOAT64: "d",
    }
    type_format = type_format_dict.get(field.datatype, "")

    return f"{endian_format}{count_format}{type_format}"


def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    r = np.array(
        [
            [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2],
        ]
    )
    return r


def main():
    rclpy.init()

    occupancy_transformer = OccupancyTransformerNode()
    rclpy.spin(occupancy_transformer)

    occupancy_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
