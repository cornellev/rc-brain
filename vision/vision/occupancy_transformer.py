import struct

import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.node
import rclpy.parameter
import rclpy.qos
import rclpy.time
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2, PointField


class OccupancyTransformerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("occupancy_transformer")

        self.declare_parameter(
            "point_cloud_topic",
            "point_cloud",
            ParameterDescriptor(
                description="The topic to subscribe to for point cloud data"
            ),
        )
        self.declare_parameter(
            "map_x_size",
            10.0,
            ParameterDescriptor(
                description="The size of the map in meters along the x-axis of base_link_frame"
            ),
        )
        self.declare_parameter(
            "map_y_size",
            10.0,
            ParameterDescriptor(
                description="The size of the map in meters along the y-axis of base_link_frame"
            ),
        )
        self.declare_parameter(
            "map_resolution",
            0.1,
            ParameterDescriptor(
                description="The resolution of the map in meters per cell"
            ),
        )
        self.declare_parameter(
            "fuzz_threshold",
            0.1,
            ParameterDescriptor(
                description="The threshold (in meters) beyond car dimensions to consider a point as an obstacle."
            ),
        )
        self.declare_parameter(
            "base_link_height",
            rclpy.Parameter.Type.DOUBLE,
            ParameterDescriptor(
                description="The height (in meters) of base_link_frame above the ground."
            ),
        )
        self.declare_parameter(
            "ride_height",
            rclpy.Parameter.Type.DOUBLE,
            ParameterDescriptor(
                description="The height (in meters) above the ground plane of the tallest traversable obstacle."
            ),
        )
        self.declare_parameter(
            "car_height",
            rclpy.Parameter.Type.DOUBLE,
            ParameterDescriptor(
                description="The height (in meters) of the tallest point on the car."
            ),
        )
        self.declare_parameter(
            "base_link_frame",
            "base_link",
            ParameterDescriptor(description="The frame_id of the base link frame."),
        )

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
        map_x_m = self.get_parameter("map_x_size").value
        map_y_m = self.get_parameter("map_y_size").value
        map_res_m = self.get_parameter("map_resolution").value

        map_x_cells = int(map_x_m / map_res_m)
        map_y_cells = int(map_y_m / map_res_m)

        base_link_height_m = self.get_parameter("base_link_height").value
        ride_height_m = self.get_parameter("ride_height").value
        car_height_m = self.get_parameter("car_height").value
        fuzz_threshold_m = self.get_parameter("fuzz_threshold").value

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
        point_cloud = point_cloud[~np.isnan(point_cloud).any(axis=1)]
        base_link_pc = point_cloud @ rotation.T + translation

        pc_z = base_link_pc[:, 2]
        filtered_pc = base_link_pc[
            (pc_z < (car_height_m - base_link_height_m + fuzz_threshold_m))
            & (pc_z > (-(base_link_height_m - ride_height_m) - fuzz_threshold_m))
        ]

        grid = np.zeros((map_x_cells, map_y_cells), dtype=np.int8)

        cell_x = filtered_pc[:, 0] / map_res_m
        cell_y = (filtered_pc[:, 1] + map_y_m / 2) / map_res_m
        cells = np.stack((cell_x, cell_y), axis=-1).astype(np.int32)

        in_range = cells[
            (
                (cells[:, 0] >= 0)
                & (cells[:, 0] < map_x_cells)
                & (cells[:, 1] >= 0)
                & (cells[:, 1] < map_y_cells)
            )
        ]
        in_range = cells

        # grid[in_range] = 255
        for x, y in in_range:
            grid[x, y] = 255

        # nav_msgs/OccupancyGrid:
        # The map data, in row-major order, starting with (0,0).
        # Cell (1, 0) will be listed second, representing the next cell in the x direction.
        # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
        # The values inside are application dependent, but frequently,
        # 0 represents unoccupied, 1 represents definitely occupied, and
        # -1 represents unknown.

        # flatten in column-major order, as we've made our rows constant x but that is not what nav_msgs/OccupancyGrid wants
        flat = grid.flatten(order="F")

        occupancy_grid = OccupancyGrid()

        occupancy_grid.header.stamp = cloud.header.stamp
        occupancy_grid.header.frame_id = base_link_frame

        occupancy_grid.info.map_load_time = cloud.header.stamp
        occupancy_grid.info.resolution = map_res_m
        occupancy_grid.info.width = map_x_cells
        occupancy_grid.info.height = map_y_cells

        origin_offset = (-map_y_cells * map_res_m) / 2
        map_origin = Pose(
            position=Point(x=0.0, y=origin_offset, z=-base_link_height_m),
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
