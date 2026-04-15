#!/usr/bin/env python3
"""
Gripper Attach Node

When a segmented pointcloud arrives:
  1. Its convex hull is added to the MoveIt planning scene as a collision object
     (frame: camera optical frame → MoveIt planning frame via TF).
  2. The nearest Gazebo model name is auto-resolved by comparing the centroid
     against live Gz world poses (offset-calibrated to the ROS TF root).
  3. When the gripper link enters attach_distance of the centroid:
     - Gz: publishes the model name to /gripper/attach (DetachableJoint)
     - MoveIt: attaches the collision object to the gripper link
  4. When the gripper moves away (2× attach_distance):
     - Gz: publishes to /gripper/detach
     - MoveIt: detaches and removes the collision object

Parameters:
  finger_link_left    (str,   default 'left_inner_finger_pad')
  finger_link_right   (str,   default 'right_inner_finger_pad')
  attach_link         (str,   default 'robotiq_140_base_link')  MoveIt attach link
  planning_frame      (str,   default 'odom')  MoveIt planning/TF root frame
  robot_gz_model      (str,   default 'summit')
  robot_tf_frame      (str,   default 'base_footprint')
  attach_distance     (float, default 0.10)  metres
  max_match_distance  (float, default 0.30)  metres
  exclude_prefixes    (str,   default 'summit,ground_plane,sun,aws_robomaker')
  gz_world            (str,   default 'world_demo')
"""

import re
import struct
import subprocess
import threading

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
import tf2_ros


# All gripper links — used as touch_links so MoveIt allows contact during grasp
_GRIPPER_LINKS = [
    'robotiq_base_link',
    'robotiq_140_base_link',
    'left_outer_knuckle',
    'left_outer_finger',
    'left_inner_knuckle',
    'left_inner_finger',
    'left_inner_finger_pad',
    'right_outer_knuckle',
    'right_outer_finger',
    'right_inner_knuckle',
    'right_inner_finger',
    'right_inner_finger_pad',
]

# Planning scene topic (MoveIt monitors this)
_PLANNING_SCENE_TOPIC = '/planning_scene'


# ---------------------------------------------------------------------------
# Gz text-proto parser
# ---------------------------------------------------------------------------

_RE_NAME  = re.compile(r'name:\s*"([^"]+)"')
_RE_FLOAT = re.compile(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?')


def _extract_blocks(text: str, keyword: str) -> list[str]:
    blocks, start, search = [], 0, keyword + ' {'
    while True:
        idx = text.find(search, start)
        if idx == -1:
            break
        depth, i = 0, idx + len(keyword) + 1
        while i < len(text):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    blocks.append(text[idx + len(keyword) + 1: i])
                    start = i + 1
                    break
            i += 1
        else:
            break
    return blocks


def _parse_pose_info(text: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Parse Gz pose dump → {name: (position xyz, quaternion xyzw)}."""
    result = {}
    for content in _extract_blocks(text, 'pose'):
        m = _RE_NAME.search(content)
        if not m:
            continue
        pos_blocks = _extract_blocks(content, 'position')
        if not pos_blocks:
            continue
        pf = _RE_FLOAT.findall(pos_blocks[0])
        if len(pf) < 3:
            continue
        pos = np.array([float(pf[0]), float(pf[1]), float(pf[2])])
        ori_blocks = _extract_blocks(content, 'orientation')
        if ori_blocks:
            of = _RE_FLOAT.findall(ori_blocks[0])
            if len(of) >= 4:
                # Gz prints x, y, z, w (in that order)
                quat = np.array([float(of[0]), float(of[1]),
                                 float(of[2]), float(of[3])])
            else:
                quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        result[m.group(1)] = (pos, quat)
    return result


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class GripperAttachNode(Node):

    def __init__(self):
        super().__init__('gripper_attach_node')

        # Finger pads are ~18 cm from the base link — use their midpoint for proximity
        self.declare_parameter('finger_link_left',   'left_inner_finger_pad')
        self.declare_parameter('finger_link_right',  'right_inner_finger_pad')
        self.declare_parameter('attach_link',        'robotiq_140_base_link')
        # planning_frame must be map (not odom) — the stored centroid must remain
        # valid in world coordinates as the robot moves toward the object.
        self.declare_parameter('planning_frame',     'map')
        self.declare_parameter('robot_gz_model',     'summit')
        self.declare_parameter('robot_tf_frame',     'base_footprint')
        self.declare_parameter('attach_distance',    0.15)
        self.declare_parameter('max_match_distance', 0.30)
        self.declare_parameter('exclude_prefixes',
                               'summit,ground_plane,sun,aws_robomaker')
        self.declare_parameter('gz_world',           'world_demo')

        self._finger_left    = self.get_parameter('finger_link_left').value
        self._finger_right   = self.get_parameter('finger_link_right').value
        self._attach_link    = self.get_parameter('attach_link').value
        self._planning_frame = self.get_parameter('planning_frame').value
        self._robot_gz_model = self.get_parameter('robot_gz_model').value
        self._robot_tf_frame = self.get_parameter('robot_tf_frame').value
        self._attach_dist    = self.get_parameter('attach_distance').value
        self._max_match_dist = self.get_parameter('max_match_distance').value
        self._gz_world       = self.get_parameter('gz_world').value
        excl = self.get_parameter('exclude_prefixes').value
        self._exclude        = [s.strip() for s in excl.split(',') if s.strip()]

        # gz_world → planning_frame full SE(3) calibration:
        # R_gz_to_ros (3x3), t_gz_to_ros (3,)
        # such that: pos_ros = R_gz_to_ros @ pos_gz + t_gz_to_ros
        self._R_gz_to_ros: np.ndarray | None = None
        self._t_gz_to_ros: np.ndarray | None = None
        self._calib_lock = threading.Lock()

        # Live Gz poses (gz world frame): name → (pos xyz, quat xyzw)
        self._model_poses_gz: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._poses_lock = threading.Lock()

        # Current object state
        self._attached          = False
        self._target_model: str | None      = None   # Gz model name
        self._collision_obj_id: str | None  = None   # MoveIt collision object id
        self._object_pos: np.ndarray | None = None   # centroid in planning_frame

        # TF
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Gz attach/detach publishers (bridged to Gazebo DetachableJoint)
        self._gz_attach_pub = self.create_publisher(String, '/gripper/attach', 5)
        self._gz_detach_pub = self.create_publisher(Empty,  '/gripper/detach', 5)

        # Explicit detach trigger from pick-and-place scripts.
        # Accept both Empty (original) and String (works reliably over rosbridge,
        # which has trouble with zero-field Empty messages in advertise/publish races).
        self.create_subscription(Empty, '/gripper/force_detach',
                                 self._force_detach_cb, 5)
        self.create_subscription(String, '/gripper/force_detach_str',
                                 lambda msg: self._force_detach_cb(Empty()), 5)

        # MoveIt planning scene publisher
        from moveit_msgs.msg import PlanningScene
        self._scene_pub = self.create_publisher(
            PlanningScene, _PLANNING_SCENE_TOPIC, 5
        )

        # Segmented pointcloud — TRANSIENT_LOCAL matches the latched publisher
        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            PointCloud2, '/segmented_pointcloud', self._pointcloud_cb, pc_qos
        )

        threading.Thread(target=self._gz_pose_reader, daemon=True).start()
        self.create_timer(0.1, self._check_proximity)

        self.get_logger().info(
            f'GripperAttachNode ready | planning_frame={self._planning_frame} '
            f'fingers={self._finger_left}/{self._finger_right} '
            f'attach_distance={self._attach_dist}m'
        )

    # ------------------------------------------------------------------
    # Background Gz pose reader + calibration
    # ------------------------------------------------------------------

    def _gz_pose_reader(self):
        topic = f'/world/{self._gz_world}/pose/info'
        try:
            proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', topic],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            )
        except FileNotFoundError:
            self.get_logger().error('`gz` binary not found — model auto-resolution disabled.')
            return

        buf = ''
        for line in proc.stdout:
            buf += line
            if line.strip() == '':
                if buf.strip():
                    poses = _parse_pose_info(buf)
                    if poses:
                        with self._poses_lock:
                            self._model_poses_gz.update(poses)
                        self._try_calibrate(poses)
                buf = ''

    def _try_calibrate(self, poses: dict[str, tuple[np.ndarray, np.ndarray]]):
        if self._robot_gz_model not in poses:
            return
        robot_gz_pos, robot_gz_quat = poses[self._robot_gz_model]
        try:
            t: TransformStamped = self._tf_buffer.lookup_transform(
                self._planning_frame, self._robot_tf_frame, rclpy.time.Time()
            )
        except Exception:
            return
        tx = t.transform.translation
        rq = t.transform.rotation
        robot_ros_pos = np.array([tx.x, tx.y, tx.z])
        R_map_base = _quat_to_rot(rq.x, rq.y, rq.z, rq.w)
        R_gz_base  = _quat_to_rot(robot_gz_quat[0], robot_gz_quat[1],
                                  robot_gz_quat[2], robot_gz_quat[3])
        # T_map_gz = T_map_base @ T_base_gz = T_map_base @ inv(T_gz_base)
        R_base_gz = R_gz_base.T
        t_base_gz = -R_base_gz @ robot_gz_pos
        R_map_gz  = R_map_base @ R_base_gz
        t_map_gz  = R_map_base @ t_base_gz + robot_ros_pos
        with self._calib_lock:
            first = self._R_gz_to_ros is None
            self._R_gz_to_ros = R_map_gz
            self._t_gz_to_ros = t_map_gz
        if first:
            self.get_logger().info(
                f'Calibrated gz→ROS SE(3): t='
                f'[{t_map_gz[0]:.3f}, {t_map_gz[1]:.3f}, {t_map_gz[2]:.3f}] '
                f'R[0]=[{R_map_gz[0,0]:.3f}, {R_map_gz[0,1]:.3f}, {R_map_gz[0,2]:.3f}]'
            )

    # ------------------------------------------------------------------
    # Pointcloud callback — update scene object and resolve model name
    # ------------------------------------------------------------------

    def _pointcloud_cb(self, msg: PointCloud2):
        pts = _pointcloud2_to_xyz(msg)
        if pts is None or len(pts) == 0:
            return

        centroid_cam = pts.mean(axis=0)

        # Use the pointcloud's own timestamp so a latched (TRANSIENT_LOCAL)
        # message is transformed with the TF that was valid when it was captured,
        # not the current (potentially moved) robot pose.
        stamp = msg.header.stamp
        # Fall back to latest TF if stamp is zero (unset)
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = rclpy.time.Time()
        else:
            stamp = rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)

        try:
            t: TransformStamped = self._tf_buffer.lookup_transform(
                self._planning_frame, msg.header.frame_id, stamp,
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            tx = t.transform.translation
            q  = t.transform.rotation
            R  = _quat_to_rot(q.x, q.y, q.z, q.w)
            t_vec = np.array([tx.x, tx.y, tx.z])
        except Exception as e:
            self.get_logger().warn(
                f'TF {msg.header.frame_id}→{self._planning_frame} failed: {e}',
                throttle_duration_sec=5.0,
            )
            return

        centroid_ros = R @ centroid_cam + t_vec
        self._object_pos = centroid_ros

        if not self._attached:
            pts_ros = (R @ pts.T).T + t_vec

            obj_id = self._target_model or 'segmented_object'
            self._update_collision_object(obj_id, pts_ros, self._planning_frame)
            self._collision_obj_id = obj_id
            self._target_model = self._find_nearest_model(centroid_ros)
            # Rename collision object once model name is known
            if self._target_model and self._collision_obj_id != self._target_model:
                self._remove_collision_object(self._collision_obj_id)
                self._update_collision_object(self._target_model, pts_ros,
                                              self._planning_frame)
                self._collision_obj_id = self._target_model

    # ------------------------------------------------------------------
    # Proximity check
    # ------------------------------------------------------------------

    def _check_proximity(self):
        if self._object_pos is None:
            return
        gripper = self._get_gripper_pos()
        if gripper is None:
            return

        dist = float(np.linalg.norm(gripper - self._object_pos))

        self.get_logger().info(
            f'finger_midpoint→object dist: {dist:.3f}m '
            f'(target={self._target_model}, attached={self._attached})',
            throttle_duration_sec=1.0,
        )

        if not self._attached:
            if self._target_model and dist < self._attach_dist:
                self._do_attach()
        # If attached, don't auto-detach based on distance — the centroid is frozen
        # at the pre-grasp position so distance grows as soon as the arm lifts.

    # ------------------------------------------------------------------
    # Attach / detach (Gz + MoveIt)
    # ------------------------------------------------------------------

    def _force_detach_cb(self, _msg):
        self.get_logger().info('force_detach received')
        if self._attached:
            self._do_detach()
        else:
            self.get_logger().warn('force_detach received but nothing is attached.')

    def _gz_publish(self, topic: str, msg_type: str, payload: str):
        """Publish directly to gz-transport via the `gz topic` CLI.

        The ros_gz_bridge ROS→GZ path is unreliable for short-lived publishes
        (lazy subscription activation + VOLATILE QoS can drop the first msg).
        Shelling out bypasses the bridge entirely and is reliable.
        """
        try:
            subprocess.run(
                ['gz', 'topic', '-t', topic, '-m', msg_type, '-p', payload],
                check=True, timeout=2.0,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
        except Exception as e:
            self.get_logger().error(f'gz topic publish {topic} failed: {e}')

    def _do_attach(self):
        # 1. Gazebo DetachableJoint — publish directly to gz-transport
        self._gz_publish('/gripper/attach', 'gz.msgs.StringMsg',
                         f'data: "{self._target_model}"')

        # 2. MoveIt planning scene — attach collision object to gripper link
        self._attach_collision_object(
            self._collision_obj_id, self._attach_link, _GRIPPER_LINKS
        )

        self._attached = True
        self.get_logger().info(
            f'Attached "{self._target_model}" '
            f'(Gz DetachableJoint + MoveIt collision object on {self._attach_link})'
        )

    def _do_detach(self):
        # 1. Gazebo DetachableJoint — publish directly to gz-transport
        self._gz_publish('/gripper/detach', 'gz.msgs.Empty', '')

        # 2. MoveIt planning scene — detach and remove collision object
        if self._collision_obj_id:
            self._detach_collision_object(self._collision_obj_id, self._attach_link)
            self._remove_collision_object(self._collision_obj_id)

        self.get_logger().info(f'Detached "{self._target_model}".')
        self._attached         = False
        self._target_model     = None
        self._collision_obj_id = None
        self._object_pos       = None

    # ------------------------------------------------------------------
    # MoveIt planning scene helpers
    # ------------------------------------------------------------------

    def _update_collision_object(self, obj_id: str, pts_ros: np.ndarray,
                                  frame_id: str):
        """Build a convex hull mesh from pts_ros and publish it to the planning scene."""
        from moveit_msgs.msg import PlanningScene

        mesh = _convex_hull_mesh(pts_ros)
        if mesh is None:
            self.get_logger().warn('Convex hull failed — not enough points.',
                                   throttle_duration_sec=2.0)
            return

        co = CollisionObject()
        co.header.frame_id = frame_id
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = obj_id
        co.operation = CollisionObject.ADD
        co.meshes = [mesh]

        # Identity pose — mesh vertices are already in frame_id
        pose = Pose()
        pose.orientation.w = 1.0
        co.mesh_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self._scene_pub.publish(scene)

    def _remove_collision_object(self, obj_id: str):
        from moveit_msgs.msg import PlanningScene

        co = CollisionObject()
        co.id = obj_id
        co.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self._scene_pub.publish(scene)

    def _attach_collision_object(self, obj_id: str, link: str,
                                  touch_links: list[str]):
        """Move object from world collision objects to robot attached objects."""
        from moveit_msgs.msg import PlanningScene

        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.touch_links = touch_links

        # Re-describe the object so MoveIt knows what shape to carry
        co = CollisionObject()
        co.id = obj_id
        co.operation = CollisionObject.ADD
        # Shape is already in the scene; just reference it by id
        aco.object = co

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        # Remove from world simultaneously
        world_co = CollisionObject()
        world_co.id = obj_id
        world_co.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [world_co]
        self._scene_pub.publish(scene)

    def _detach_collision_object(self, obj_id: str, link: str):
        """Move object from attached back to world (before removing)."""
        from moveit_msgs.msg import PlanningScene

        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self._scene_pub.publish(scene)

    # ------------------------------------------------------------------
    # Model name resolution
    # ------------------------------------------------------------------

    def _find_nearest_model(self, centroid_ros: np.ndarray) -> str | None:
        with self._calib_lock:
            R = self._R_gz_to_ros
            t = self._t_gz_to_ros
        if R is None or t is None:
            self.get_logger().warn(
                'gz→ROS calibration not ready yet.', throttle_duration_sec=5.0
            )
            return None

        with self._poses_lock:
            poses_gz = dict(self._model_poses_gz)

        best_name, best_dist = None, self._max_match_dist
        closest_any_name, closest_any_dist = None, float('inf')
        for name, (pos_gz, _q) in poses_gz.items():
            if any(name.startswith(p) for p in self._exclude):
                continue
            pos_ros = R @ pos_gz + t
            d = float(np.linalg.norm(pos_ros - centroid_ros))
            if d < closest_any_dist:
                closest_any_dist, closest_any_name = d, name
            if d < best_dist:
                best_dist, best_name = d, name

        if best_name and best_name != self._target_model:
            self.get_logger().info(
                f'Auto-resolved target: "{best_name}" ({best_dist:.3f} m from centroid)'
            )
        elif best_name is None:
            self.get_logger().warn(
                f'No model within {self._max_match_dist:.2f}m. '
                f'Closest: "{closest_any_name}" at {closest_any_dist:.3f}m. '
                f'centroid={centroid_ros.tolist()}',
                throttle_duration_sec=5.0,
            )
        return best_name

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_gripper_pos(self) -> np.ndarray | None:
        """Return midpoint between the two inner finger pads in planning_frame."""
        try:
            tl = self._tf_buffer.lookup_transform(
                self._planning_frame, self._finger_left, rclpy.time.Time()
            ).transform.translation
            tr = self._tf_buffer.lookup_transform(
                self._planning_frame, self._finger_right, rclpy.time.Time()
            ).transform.translation
            return np.array([
                (tl.x + tr.x) * 0.5,
                (tl.y + tr.y) * 0.5,
                (tl.z + tr.z) * 0.5,
            ])
        except Exception:
            return None


# ---------------------------------------------------------------------------
# Pure functions
# ---------------------------------------------------------------------------

def _convex_hull_mesh(pts: np.ndarray) -> Mesh | None:
    """Compute convex hull of pts (Nx3, planning_frame) → shape_msgs/Mesh."""
    if len(pts) < 4:
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    try:
        hull, _ = pcd.compute_convex_hull()
    except Exception:
        return None

    verts = np.asarray(hull.vertices)
    tris  = np.asarray(hull.triangles)
    if len(verts) == 0 or len(tris) == 0:
        return None

    mesh = Mesh()
    for v in verts:
        p = Point()
        p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
        mesh.vertices.append(p)
    for t in tris:
        tri = MeshTriangle()
        tri.vertex_indices = [int(t[0]), int(t[1]), int(t[2])]
        mesh.triangles.append(tri)
    return mesh


def _quat_to_rot(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray | None:
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return None
    fx, fy, fz = fields['x'].offset, fields['y'].offset, fields['z'].offset
    step = msg.point_step
    data = msg.data
    n = len(data) // step
    pts = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        b = i * step
        pts[i, 0] = struct.unpack_from('<f', data, b + fx)[0]
        pts[i, 1] = struct.unpack_from('<f', data, b + fy)[0]
        pts[i, 2] = struct.unpack_from('<f', data, b + fz)[0]
    return pts[np.isfinite(pts).all(axis=1)]


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GripperAttachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
