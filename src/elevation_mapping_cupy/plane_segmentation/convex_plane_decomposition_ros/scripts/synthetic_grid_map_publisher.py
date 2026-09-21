#!/usr/bin/env python3
"""Publish an explicitly synthetic GridMap for offline verification."""

import math

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def to_multi_array(array: np.ndarray) -> Float32MultiArray:
    message = Float32MultiArray()
    rows, cols = array.shape
    message.layout.dim = [
        MultiArrayDimension(
            label="column_index", size=cols, stride=rows * cols
        ),
        MultiArrayDimension(label="row_index", size=rows, stride=rows),
    ]
    message.data = array.astype(np.float32).flatten(order="F").tolist()
    return message


class SyntheticGridMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_plane_grid_map_publisher")
        self.declare_parameter(
            "topic", "/elevation_mapping_node/elevation_map_raw"
        )
        self.declare_parameter("frame_id", "global")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("length_x", 3.0)
        self.declare_parameter("length_y", 3.0)
        self.declare_parameter("resolution", 0.04)

        self._topic = str(self.get_parameter("topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._length_x = float(self.get_parameter("length_x").value)
        self._length_y = float(self.get_parameter("length_y").value)
        self._resolution = float(self.get_parameter("resolution").value)
        if self._rate_hz <= 0.0 or self._resolution <= 0.0:
            raise ValueError("rate_hz and resolution must be positive")

        self._publisher = self.create_publisher(GridMap, self._topic, 1)
        self._message = self._make_message()
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish)
        rows = (
            len(self._message.data[0].data)
            // self._message.data[0].layout.dim[0].size
        )
        cols = self._message.data[0].layout.dim[0].size
        self.get_logger().info(
            "Publishing synthetic test GridMap only: "
            f"{rows}x{cols} cells at {self._rate_hz:.3f} Hz"
        )

    def _make_message(self) -> GridMap:
        rows = max(1, int(round(self._length_x / self._resolution)))
        cols = max(1, int(round(self._length_y / self._resolution)))
        x = np.linspace(
            self._length_x / 2.0,
            -self._length_x / 2.0,
            rows,
            dtype=np.float32,
        )
        y = np.linspace(
            self._length_y / 2.0,
            -self._length_y / 2.0,
            cols,
            dtype=np.float32,
        )
        xx, yy = np.meshgrid(x, y, indexing="ij")
        elevation = 0.12 + 0.03 * xx - 0.02 * yy
        variance = np.full((rows, cols), 0.0025, dtype=np.float32)

        # Synthetic missing-data patch; this is not sensor evidence.
        hole_radius = max(1, int(math.ceil(0.08 / self._resolution)))
        center_row = rows // 2
        center_col = cols // 2
        elevation[
            center_row - hole_radius:center_row + hole_radius,
            center_col - hole_radius:center_col + hole_radius,
        ] = np.nan

        message = GridMap()
        message.header.frame_id = self._frame_id
        message.info.resolution = self._resolution
        message.info.length_x = rows * self._resolution
        message.info.length_y = cols * self._resolution
        message.info.pose.orientation.w = 1.0
        message.layers = ["elevation", "variance"]
        message.basic_layers = ["elevation"]
        message.data = [to_multi_array(elevation), to_multi_array(variance)]
        return message

    def _publish(self) -> None:
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def main() -> None:
    rclpy.init()
    node = SyntheticGridMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
