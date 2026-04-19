#!/usr/bin/env python3

"""
 @maintainer: Azeez Adebayo

Install: pip install pyrealsense2
         pip install opencv-python

OpenCV and Numpy Point cloud Software Renderer for realsense camera 

Usage:
------
Mouse: 
    Drag with left button to rotate around pivot (thick small axes), 
    with right button to translate and the wheel to zoom.

Keyboard: 
    [r]     Reset View
    [s]     Save PNG (./out.png)
    [e]     Export points to ply (./out.ply)
    [q\ESC] Quit
"""

# Migrated to ROS 2 (rclpy) from ROS 1 (rospy)

import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge

class AppState:
    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

state = AppState()

# Global variables for pipeline and out
pipeline = None
out = None
pc = None
decimate = None
colorizer = None

def mouse_cb(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        state.mouse_btns[0] = True
    if event == cv2.EVENT_LBUTTONUP:
        state.mouse_btns[0] = False
    if event == cv2.EVENT_RBUTTONDOWN:
        state.mouse_btns[1] = True
    if event == cv2.EVENT_RBUTTONUP:
        state.mouse_btns[1] = False
    if event == cv2.EVENT_MBUTTONDOWN:
        state.mouse_btns[2] = True
    if event == cv2.EVENT_MBUTTONUP:
        state.mouse_btns[2] = False
    if event == cv2.EVENT_MOUSEMOVE:
        h, w = out.shape[:2]
        dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]
        if state.mouse_btns[0]:
            state.yaw += float(dx) / w * 2
            state.pitch -= float(dy) / h * 2
        elif state.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            state.translation -= np.dot(state.rotation, dp)
        elif state.mouse_btns[2]:
            dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
            state.translation[2] += dz
            state.distance -= dz
    if event == cv2.EVENT_MOUSEWHEEL:
        dz = math.copysign(0.1, flags)
        state.translation[2] += dz
        state.distance -= dz
    state.prev_mouse = (x, y)

def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w
    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)
    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj

def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation

def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)

def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    for i in range(0, n+1):
        x = -s2 + i*s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
            view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n+1):
        z = -s2 + i*s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
            view(pos + np.dot((s2, 0, z), rotation)), color)

def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
        np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
        np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
        np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)

def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 0])
    w, h = intrinsics.width, intrinsics.height
    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p
        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)
        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)

def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))
    if state.scale:
        proj *= 0.5**state.decimate
    h, w = out.shape[:2]
    j, i = proj.astype(np.uint32).T
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm
    cw, ch = color.shape[:2][::-1]
    if painter:
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)
    out[i[m], j[m]] = color[u[m], v[m]]

# --------------------------
# ROS 2 Node
# --------------------------

class RenderRSPC(Node):
    def __init__(self):
        super().__init__('render_rs_pc')
        
        # RealSense initialization
        global pipeline, pc, decimate, colorizer, out
        pipeline = rs.pipeline()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            self.get_logger().error("The demo requires Depth camera with Color sensor")
            return

        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        pipeline.start(config)
        
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height
        
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
        colorizer = rs.colorizer()
        
        cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(state.WIN_NAME, w, h)
        cv2.setMouseCallback(state.WIN_NAME, mouse_cb)
        out = np.empty((h, w, 3), dtype=np.uint8)

        self.scan = True
        self.bridge = CvBridge()

        # ROS 2 publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/kinect_camera/points', 10)
        self.color_pub = self.create_publisher(Image, '/kinect_camera/color', 10)
        self.depth_pub = self.create_publisher(Image, '/kinect_camera/depth', 10)

        # ROS 2 service
        self.srv = self.create_service(SetBool, '/mcfly_scan/scan', self.scan_n_plan_srv_cb)

        # ROS 2 timer (20Hz)
        self.timer = self.create_timer(0.05, self.scan_surface)

    def scan_n_plan_srv_cb(self, request, response):
        if request.data:
            self.scan = True
            self.get_logger().info("Call to start scan was successful.")
        else:
            self.scan = False
            try:
                # pipeline.stop() # Usually we don't want to stop the whole pipeline if we just want to stop "scanning" but keep the node alive
                # cv2.destroyAllWindows()
                self.get_logger().info("Call to stop scan was successful.")
            except Exception as e:
                self.get_logger().warn(f"Error stopping: {e}")
        response.success = True
        response.message = "Scan request processed"
        return response

    def create_pointcloud2(self, verts, header):
        """Convert numpy array of vertices to sensor_msgs/PointCloud2."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        data = verts.tobytes()

        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(verts)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = point_step
        pc2.row_step = point_step * len(verts)
        pc2.data = data
        pc2.is_dense = True
        return pc2

    def scan_surface(self):
        if not self.scan:
            return
        
        try:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame() 
            color_frame = frames.get_color_frame() 
            depth_frame = decimate.process(depth_frame)
            depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height
            
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            if state.color:
                mapped_frame, color_source = color_frame, color_image
            else:
                mapped_frame, color_source = depth_frame, depth_colormap

            points_rs = pc.calculate(depth_frame)
            pc.map_to(mapped_frame)
            v, t = points_rs.get_vertices(), points_rs.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)
            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)

            header = Image().header # Use a dummy message to get header
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera_depth_frame"
            
            pc2_msg = self.create_pointcloud2(verts, header)
            self.pc_pub.publish(pc2_msg)

            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header = header
            self.color_pub.publish(color_msg)

            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
            depth_msg.header = header
            self.depth_pub.publish(depth_msg)

            # Visualization
            now = time.time()
            out.fill(0)
            grid(out, (0, 0.5, 1), size=1, n=10)
            frustum(out, depth_intrinsics)
            axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)

            if not state.scale or out.shape[:2] == (h, w):
                pointcloud(out, verts, texcoords, color_source)
            else:
                tmp = np.zeros((h, w, 3), dtype=np.uint8)
                pointcloud(tmp, verts, texcoords, color_source)
                tmp = cv2.resize(tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
                np.putmask(out, tmp > 0, tmp)

            if any(state.mouse_btns):
                axes(out, view(state.pivot), state.rotation, thickness=4)

            dt = time.time() - now
            cv2.setWindowTitle(
                state.WIN_NAME, 
                "press [e] export ply | [r] reset | RealSense info: (%dx%d) %dFPS (%.2fms) %s" %
                (w, h, 1.0/dt if dt > 0 else 0, dt*1000, "PAUSED" if state.paused else ""))
            cv2.imshow(state.WIN_NAME, out)
            key = cv2.waitKey(1)

            if key == ord("r"):
                state.reset()
            if key == ord("s"):
                cv2.imwrite('./out.png', out)
                self.get_logger().info("Saved PNG to ./out.png")
            if key == ord("e"):
                points_rs.export_to_ply('./out.ply', mapped_frame)
                self.get_logger().info("Exported PLY to ./out.ply")
            if key in [ord("q"), 27]: # q or ESC
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error in scan_surface: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RenderRSPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if pipeline:
            pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
