import sys
import math
import struct
import open3d as o3d
import os
from sensor_msgs.msg import PointField

# Data type mapping for PointCloud2 fields
_DATATYPES = {
    PointField.INT8: ('b', 1),
    PointField.UINT8: ('B', 1),
    PointField.INT16: ('h', 2),
    PointField.UINT16: ('H', 2),
    PointField.INT32: ('i', 4),
    PointField.UINT32: ('I', 4),
    PointField.FLOAT32: ('f', 4),
    PointField.FLOAT64: ('d', 8)
}

class PointCloudScanner:
    """Handles PointCloud accumulation, visualization, and saving."""
    def __init__(self, output_path="point_cloud.ply"):
        self.pcd = o3d.geometry.PointCloud()
        self.output_path = output_path
        self.vis = None
        self.is_active = False

    def start_session(self):
        """Initializes the Open3D visualization window."""
        self.is_active = True
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Surface Scan", width=350, height=350)
        self.pcd = o3d.geometry.PointCloud() # Clear buffer

    def stop_session(self):
        """Closes the window and saves the accumulated cloud."""
        if not self.is_active:
            return
        
        self.is_active = False
        if len(self.pcd.points) > 0:
            o3d.io.write_point_cloud(self.output_path, self.pcd)
            print(f"[surface_scan] Saved point cloud to {self.output_path}")
        
        if self.vis:
            self.vis.destroy_window()
            self.vis = None

    def update(self, points):
        """Update the visualization and accumulated cloud with new points."""
        if not self.is_active or not points:
            return
        
        # In this implementation, we replace the cloud with the latest frame for visualization
        # but in a real 'accumulation' scenario, we might want to merge them.
        # Original logic: self.pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        
        new_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        
        self.vis.remove_geometry(self.pcd)
        self.pcd = new_pcd
        self.vis.add_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

def get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'
    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(f'Skipping unknown PointField datatype [{field.datatype}]', file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length
    return fmt

def read_points(cloud, field_names=None, skip_nans=False):
    """
    Decodes sensor_msgs/PointCloud2 into a generator of points.
    """
    fmt = get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data = (
        cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
    )
    unpack_from = struct.Struct(fmt).unpack_from
    isnan = math.isnan

    if skip_nans:
        for v in range(height):
            offset = row_step * v
            for u in range(width):
                p = unpack_from(data, offset)
                if not any(isnan(pv) for pv in p):
                    yield p
                offset += point_step
    else:
        for v in range(height):
            offset = row_step * v
            for u in range(width):
                yield unpack_from(data, offset)
                offset += point_step
