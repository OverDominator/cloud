#!/usr/bin/env python3
import math
import os
import threading
import time

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String, UInt32
from visualization_msgs.msg import Marker, MarkerArray


def camera_intrinsics(width, height, vertical_fov, camera_info=None):
    """Unity's ordinary perspective camera has square pixels: fx == fy."""
    if camera_info is not None:
        info_width, info_height, calibration = camera_info
        sx, sy = width / float(info_width), height / float(info_height)
        return calibration[0] * sx, calibration[4] * sy, calibration[2] * sx, calibration[5] * sy
    focal = height / (2.0 * math.tan(vertical_fov / 2.0))
    return focal, focal, width / 2.0, height / 2.0


class VictimPointCloudLocalizer:
    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", "thermal_camera_link")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.image_width = int(rospy.get_param("~image_width", 320))
        self.image_height = int(rospy.get_param("~image_height", 180))
        self.vertical_fov = math.radians(
            float(rospy.get_param("~vertical_fov_degrees", 60.0))
        )
        self.camera_info = None
        self.fx, self.fy, self.cx, self.cy = camera_intrinsics(
            self.image_width, self.image_height, self.vertical_fov
        )
        self.timeout = float(rospy.get_param("~observation_timeout", 0.75))
        self.margin = int(rospy.get_param("~box_margin_pixels", 3))
        self.min_range = float(rospy.get_param("~min_range", 0.4))
        self.max_range = float(rospy.get_param("~max_range", 30.0))
        self.depth_band = float(
            rospy.get_param("~front_surface_depth_band", 0.8)
        )
        self.min_lidar_points = max(
            1, int(rospy.get_param("~min_lidar_points", 2))
        )
        self.marker_scale = float(rospy.get_param("~victim_marker_scale", 0.5))

        # Depth Anything V2 supplies dense *relative* depth.  Sparse LiDAR
        # samples projected into the same image convert it to metric depth.
        self.use_depth_anything = bool(
            rospy.get_param("~use_depth_anything", True)
        )
        self.depth_model_name = rospy.get_param(
            "~depth_model_name", "depth-anything/Depth-Anything-V2-Small-hf"
        )
        self.depth_model_path = os.path.expanduser(
            rospy.get_param("~depth_model_path", "")
        )
        self.depth_local_files_only = bool(
            rospy.get_param("~depth_local_files_only", True)
        )
        self.depth_device_name = rospy.get_param("~depth_device", "auto")
        self.depth_rate = max(
            0.1, float(rospy.get_param("~depth_inference_rate", 0.4))
        )
        self.depth_cpu_threads = max(
            1, int(rospy.get_param("~depth_cpu_threads", 2))
        )
        self.depth_input_width = max(
            112, int(rospy.get_param("~depth_input_width", 294))
        )
        self.depth_input_height = max(
            112, int(rospy.get_param("~depth_input_height", 168))
        )
        self.depth_cache_max_age = max(
            0.0, float(rospy.get_param("~depth_cache_max_age", 0.8))
        )
        self.depth_event_triggered = bool(
            rospy.get_param("~depth_event_triggered", True)
        )
        self.depth_episode_reset_seconds = max(
            0.2, float(rospy.get_param("~depth_episode_reset_seconds", 1.0))
        )
        self.depth_max_inferences_per_episode = max(
            1, int(rospy.get_param("~depth_max_inferences_per_episode", 2))
        )
        self.depth_min_anchors = max(
            8, int(rospy.get_param("~depth_min_anchor_points", 24))
        )
        self.depth_max_fit_error = float(
            rospy.get_param("~depth_max_fit_error", 1.5)
        )
        self.depth_lidar_weight = float(
            rospy.get_param("~depth_direct_lidar_weight", 0.65)
        )
        self.depth_lidar_weight = float(
            np.clip(self.depth_lidar_weight, 0.0, 1.0)
        )
        self.depth_model = None
        self.depth_processor = None
        self.depth_torch = None
        self.depth_device = "cpu"
        self.depth_load_attempted = False
        self.last_depth_time = 0.0
        self.last_depth = None
        self.last_depth_stamp = rospy.Time(0)
        self.depth_episode_active = False
        self.depth_empty_since = None
        self.depth_inference_requested = False
        self.depth_episode_inferences = 0
        self.depth_episode_id = 0
        self.observation_lock = threading.RLock()
        self.depth_lock = threading.Lock()
        self.depth_worker_active = False
        self.bridge = CvBridge()

        self.boxes = []
        self.box_stamp = rospy.Time(0)
        self.box_received_at = rospy.Time(0)
        self.mask = None
        self.mask_stamp = rospy.Time(0)
        self.image = None
        self.image_stamp = rospy.Time(0)
        self.listener = tf.TransformListener()

        self.pose_publisher = rospy.Publisher(
            "/victim_detection/map_poses", PoseArray, queue_size=1
        )
        self.count_publisher = rospy.Publisher(
            "/victim_detection/localized_count", UInt32, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            "/victim_detection/map_markers", MarkerArray, queue_size=1
        )
        self.depth_publisher = rospy.Publisher(
            rospy.get_param(
                "~relative_depth_topic", "/victim_detection/relative_depth"
            ),
            Image,
            queue_size=1,
        )
        self.fusion_status_publisher = rospy.Publisher(
            rospy.get_param(
                "~depth_fusion_status_topic", "/victim_detection/depth_status"
            ),
            String,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(
            rospy.get_param("~camera_info_topic", "/thermal/camera_info"),
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~boxes_stamped_topic", "/victim_detection/boxes_stamped"
            ),
            PolygonStamped,
            self.boxes_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~mask_topic", "/victim_detection/mask"),
            Image,
            self.mask_callback,
            queue_size=1,
            buff_size=2**20,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/thermal/image_raw"),
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**22,
        )
        rospy.Subscriber(
            rospy.get_param("~pointcloud_topic", "/registered_scan"),
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.loginfo(
            "Victim depth-fusion localizer ready: fx=%.2f fy=%.2f camera=%s depth_anything=%s",
            self.fx,
            self.fy,
            self.camera_frame,
            self.use_depth_anything,
        )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            ).copy()
            with self.observation_lock:
                self.image = image
                self.image_stamp = message.header.stamp
        except Exception as error:
            rospy.logwarn_throttle(2.0, "Victim depth image conversion failed: %s", error)

    def boxes_callback(self, message):
        with self.observation_lock:
            self._update_boxes(message)

    def camera_info_callback(self, message):
        if message.width > 0 and message.height > 0 and message.K[0] > 0 and message.K[4] > 0:
            with self.observation_lock:
                self.camera_info = (message.width, message.height, tuple(message.K))

    def _update_boxes(self, message):
        points = message.polygon.points
        new_boxes = [
            (points[index].x, points[index].y,
             points[index + 1].x, points[index + 1].y)
            for index in range(0, len(points) - 1, 2)
        ]
        now = rospy.Time.now()
        if new_boxes:
            absence_was_long = (
                self.depth_empty_since is not None
                and (now - self.depth_empty_since).to_sec()
                >= self.depth_episode_reset_seconds
            )
            if not self.depth_episode_active or absence_was_long:
                self.depth_episode_active = True
                self.depth_episode_inferences = 0
                self.depth_inference_requested = True
                self.depth_episode_id += 1
                self.last_depth = None
            self.depth_empty_since = None
        else:
            if self.depth_empty_since is None:
                self.depth_empty_since = now
        self.boxes = new_boxes
        self.box_stamp = message.header.stamp
        self.box_received_at = now

    def mask_callback(self, message):
        if message.encoding not in ("mono8", "8UC1"):
            rospy.logwarn_throttle(
                2.0, "Unsupported victim mask encoding: %s", message.encoding
            )
            return
        raw = np.frombuffer(message.data, dtype=np.uint8)
        expected = int(message.height * message.step)
        if raw.size < expected:
            return
        mask = raw[:expected].reshape(
            int(message.height), int(message.step)
        )[:, : int(message.width)].copy()
        with self.observation_lock:
            self.mask = mask
            self.mask_stamp = message.header.stamp

    @staticmethod
    def transform_matrix(translation, quaternion):
        matrix = tf.transformations.quaternion_matrix(quaternion)
        matrix[0:3, 3] = np.asarray(translation)
        return matrix

    def publish_empty(self):
        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = self.output_frame
        self.pose_publisher.publish(poses)
        self.count_publisher.publish(UInt32(data=0))
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)
        self.marker_publisher.publish(markers)

    def load_depth_model(self):
        if self.depth_load_attempted:
            return self.depth_model is not None
        self.depth_load_attempted = True
        if not self.use_depth_anything:
            self.fusion_status_publisher.publish(String(data="lidar_only: disabled"))
            return False
        try:
            import torch
            from transformers import AutoImageProcessor, AutoModelForDepthEstimation

            torch.set_num_threads(self.depth_cpu_threads)
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:
                pass

            source = self.depth_model_path if self.depth_model_path else self.depth_model_name
            self.depth_device = (
                "cuda"
                if self.depth_device_name == "auto" and torch.cuda.is_available()
                else ("cpu" if self.depth_device_name == "auto" else self.depth_device_name)
            )
            rospy.loginfo("Loading Depth Anything V2 from %s on %s", source, self.depth_device)
            self.depth_processor = AutoImageProcessor.from_pretrained(
                source, local_files_only=self.depth_local_files_only
            )
            self.depth_model = AutoModelForDepthEstimation.from_pretrained(
                source, local_files_only=self.depth_local_files_only
            )
            self.depth_model.to(self.depth_device).eval()
            self.depth_torch = torch
            self.fusion_status_publisher.publish(
                String(data="depth_anything_ready: %s" % self.depth_device)
            )
            rospy.loginfo("Depth Anything V2 ready on %s", self.depth_device)
            return True
        except Exception as error:
            self.depth_model = None
            self.fusion_status_publisher.publish(
                String(data="lidar_only: depth model unavailable")
            )
            rospy.logerr(
                "Depth Anything V2 unavailable (%s). LiDAR-only localization remains active.",
                error,
            )
            return False

    def infer_relative_depth(self, image, image_stamp, context=None):
        """Submit at most one bounded job; the LiDAR callback never waits for it."""
        if image is None or not self.use_depth_anything:
            return None
        with self.depth_lock:
            if (
                self.last_depth is not None
                and abs((image_stamp - self.last_depth_stamp).to_sec()) <= 0.01
                and time.monotonic() - self.last_depth_time <= self.depth_cache_max_age
            ):
                return self.last_depth
            # A relative-depth image is aligned only with its original camera
            # frame. Reusing it on a newer view shifts people onto background walls.
            if self.depth_worker_active or time.monotonic() - self.last_depth_time < 1.0 / self.depth_rate:
                return None
            with self.observation_lock:
                if self.depth_event_triggered and (
                    not self.depth_inference_requested
                    or self.depth_episode_inferences >= self.depth_max_inferences_per_episode
                ):
                    return None
                episode_id = self.depth_episode_id
                if self.depth_event_triggered:
                    self.depth_episode_inferences += 1
            self.depth_worker_active = True
        worker = threading.Thread(
            target=self._depth_worker,
            args=(image.copy(), image_stamp, episode_id, context),
            name="victim_depth_inference",
            daemon=True,
        )
        worker.start()
        return None

    def _depth_worker(self, image, image_stamp, episode_id, context=None):
        try:
            if not self.load_depth_model():
                return
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            inputs = self.depth_processor(
                images=rgb,
                return_tensors="pt",
                size={
                    "height": self.depth_input_height,
                    "width": self.depth_input_width,
                },
            )
            inputs = {key: value.to(self.depth_device) for key, value in inputs.items()}
            with self.depth_torch.no_grad():
                output = self.depth_model(**inputs).predicted_depth
                output = self.depth_torch.nn.functional.interpolate(
                    output.unsqueeze(1),
                    size=rgb.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
            depth = output.detach().float().cpu().numpy()
            if not np.all(np.isfinite(depth)):
                depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            with self.depth_lock:
                if episode_id != self.depth_episode_id:
                    return
                self.last_depth = depth
                self.last_depth_stamp = image_stamp
            if context is not None and (rospy.Time.now() - image_stamp).to_sec() <= 3.0:
                # CPU inference can outlast several camera frames. Fuse only
                # with the original mask, LiDAR projection and capture pose.
                self.publish_delayed_fusion(depth, image_stamp, context, episode_id)
            normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
            debug_message = self.bridge.cv2_to_imgmsg(colored, encoding="bgr8")
            debug_message.header.stamp = image_stamp
            debug_message.header.frame_id = self.camera_frame
            self.depth_publisher.publish(debug_message)
        except Exception as error:
            rospy.logwarn_throttle(2.0, "Depth Anything inference failed: %s", error)
            self.fusion_status_publisher.publish(String(data="lidar_only: inference failed"))
        finally:
            with self.depth_lock:
                self.last_depth_time = time.monotonic()
                self.depth_worker_active = False

    def publish_delayed_fusion(self, depth, stamp, context, episode_id):
        mask, boxes, u, v, forward, in_image, world_from_camera, intrinsics = context
        if depth.shape != mask.shape:
            return
        calibration = self.calibrate_relative_depth(depth, u, v, forward, in_image)
        if calibration is None:
            return
        fx, fy, cx, cy = intrinsics
        points = []
        for x0, y0, x1, y1 in boxes:
            x0, y0 = max(0, int(x0)), max(0, int(y0))
            x1, y1 = min(mask.shape[1], int(x1) + 1), min(mask.shape[0], int(y1) + 1)
            rows, cols = np.nonzero(mask[y0:y1, x0:x1] > 0)
            if rows.size < 5:
                continue
            rows, cols = rows + y0, cols + x0
            distance = self.metric_from_relative(float(np.median(depth[rows, cols])), calibration)
            if not math.isfinite(distance) or not self.min_range <= distance <= self.max_range:
                continue
            px, py = float(np.median(cols)), float(np.median(rows))
            point = world_from_camera @ np.array([
                distance, (cx - px) * distance / fx, (cy - py) * distance / fy, 1.0])
            points.append(point[:3])
        with self.observation_lock:
            if episode_id != self.depth_episode_id:
                return
            if points:
                self.depth_inference_requested = False
                self.publish_results(points, stamp)

    @staticmethod
    def robust_affine(x_values, y_values):
        keep = np.isfinite(x_values) & np.isfinite(y_values)
        x_values = x_values[keep]
        y_values = y_values[keep]
        if x_values.size < 3 or np.ptp(x_values) < 1e-6:
            return None
        coefficients = None
        for _ in range(5):
            design = np.column_stack((x_values, np.ones(x_values.size)))
            coefficients = np.linalg.lstsq(design, y_values, rcond=None)[0]
            residual = np.abs(design @ coefficients - y_values)
            median = float(np.median(residual))
            mad = float(np.median(np.abs(residual - median))) + 1e-6
            next_keep = residual <= median + 3.5 * mad
            if next_keep.sum() < 3 or next_keep.all():
                break
            x_values = x_values[next_keep]
            y_values = y_values[next_keep]
        prediction = coefficients[0] * x_values + coefficients[1]
        error = float(np.median(np.abs(prediction - y_values)))
        return coefficients, error, x_values.size

    def calibrate_relative_depth(self, relative_depth, pixel_u, pixel_v, forward, in_image):
        indices = np.flatnonzero(in_image)
        if indices.size < self.depth_min_anchors:
            return None
        # Keep the nearest return at each pixel to prevent rear walls leaking
        # through foreground surfaces.
        nearest = {}
        for index in indices:
            key = int(pixel_v[index]) * relative_depth.shape[1] + int(pixel_u[index])
            value = float(forward[index])
            if key not in nearest or value < nearest[key][0]:
                nearest[key] = (value, index)
        anchor_indices = np.asarray([entry[1] for entry in nearest.values()], dtype=np.int64)
        if anchor_indices.size < self.depth_min_anchors:
            return None
        relative = relative_depth[pixel_v[anchor_indices], pixel_u[anchor_indices]].astype(np.float64)
        metric = forward[anchor_indices].astype(np.float64)
        candidates = []
        direct = self.robust_affine(relative, metric)
        if direct is not None:
            candidates.append(("linear", direct, False))
        inverse_input = 1.0 / np.maximum(relative, 1e-6)
        inverse = self.robust_affine(inverse_input, metric)
        if inverse is not None:
            candidates.append(("inverse", inverse, True))
        if not candidates:
            return None
        mode, result, use_inverse = min(candidates, key=lambda item: item[1][1])
        coefficients, error, count = result
        if count < self.depth_min_anchors or error > self.depth_max_fit_error:
            rospy.logwarn_throttle(
                2.0, "Depth/LiDAR calibration rejected: anchors=%d median_error=%.2fm", count, error
            )
            return None
        self.fusion_status_publisher.publish(
            String(data="fused: %s anchors=%d error=%.2fm" % (mode, count, error))
        )
        return coefficients, use_inverse, error, count

    @staticmethod
    def metric_from_relative(relative, calibration):
        coefficients, use_inverse, _, _ = calibration
        value = 1.0 / max(float(relative), 1e-6) if use_inverse else float(relative)
        return float(coefficients[0] * value + coefficients[1])

    def cloud_callback(self, message):
        # Keep the short LiDAR projection coherent across image/mask callbacks.
        # Expensive neural inference is on a separate worker, outside this lock.
        with self.observation_lock:
            self._cloud_callback_locked(message)

    def _cloud_callback_locked(self, message):
        if not self.boxes or self.mask is None:
            self.publish_empty()
            return
        if (rospy.Time.now() - self.box_received_at).to_sec() > self.timeout:
            self.publish_empty()
            return
        if abs((self.mask_stamp - self.box_stamp).to_sec()) > 0.01:
            return
        self.fx, self.fy, self.cx, self.cy = camera_intrinsics(
            self.mask.shape[1], self.mask.shape[0], self.vertical_fov, self.camera_info)

        try:
            camera_translation, camera_quaternion = self.listener.lookupTransform(
                self.camera_frame, message.header.frame_id, self.box_stamp
            )
            output_translation, output_quaternion = self.listener.lookupTransform(
                self.output_frame, message.header.frame_id, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
            # Latest-pose fallback incorrectly projects old images while turning.
            rospy.logwarn_throttle(2.0, "Victim localization TF unavailable: %s", error)
            return

        points = np.asarray(
            list(
                point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
            ),
            dtype=np.float64,
        )
        if points.size == 0:
            self.publish_empty()
            return

        homogeneous = np.column_stack((points, np.ones(points.shape[0])))
        camera_from_cloud = self.transform_matrix(
            camera_translation, camera_quaternion
        )
        output_from_cloud = self.transform_matrix(
            output_translation, output_quaternion
        )
        camera_points = (camera_from_cloud @ homogeneous.T).T[:, :3]
        forward = camera_points[:, 0]
        valid_depth = (forward >= self.min_range) & (forward <= self.max_range)
        u = self.cx - self.fx * camera_points[:, 1] / np.maximum(forward, 1e-6)
        v = self.cy - self.fy * camera_points[:, 2] / np.maximum(forward, 1e-6)
        pixel_u = np.rint(u).astype(np.int32)
        pixel_v = np.rint(v).astype(np.int32)
        in_image = (
            valid_depth
            & (pixel_u >= 0)
            & (pixel_u < self.mask.shape[1])
            & (pixel_v >= 0)
            & (pixel_v < self.mask.shape[0])
        )
        yellow_pixel = np.zeros(points.shape[0], dtype=bool)
        valid_indices = np.flatnonzero(in_image)
        yellow_pixel[valid_indices] = (
            self.mask[pixel_v[valid_indices], pixel_u[valid_indices]] > 0
        )

        relative_depth = None
        calibration = None
        if self.image is not None and abs((self.image_stamp - self.box_stamp).to_sec()) < 0.1:
            context = (self.mask.copy(), tuple(self.boxes), pixel_u, pixel_v,
                       forward, in_image,
                       output_from_cloud @ np.linalg.inv(camera_from_cloud),
                       (self.fx, self.fy, self.cx, self.cy))
            relative_depth = self.infer_relative_depth(self.image, self.image_stamp, context)
            if relative_depth is not None and relative_depth.shape == self.mask.shape:
                calibration = self.calibrate_relative_depth(
                    relative_depth, pixel_u, pixel_v, forward, in_image
                )
                if self.depth_event_triggered:
                    # A valid metric calibration completes this visual event.
                    # If calibration failed, permit one delayed retry only.
                    self.depth_inference_requested = calibration is None
                    if (calibration is None
                            and self.depth_episode_inferences
                            >= self.depth_max_inferences_per_episode):
                        self.depth_inference_requested = False

        localized_points = []
        for x_min, y_min, x_max, y_max in self.boxes:
            inside = (
                yellow_pixel
                & (u >= x_min - self.margin)
                & (u <= x_max + self.margin)
                & (v >= y_min - self.margin)
                & (v <= y_max + self.margin)
            )
            candidate_indices = np.flatnonzero(inside)
            lidar_position = None
            if candidate_indices.size >= self.min_lidar_points:
                candidate_depths = forward[candidate_indices]
                front_depth = float(np.percentile(candidate_depths, 10.0))
                surface_indices = candidate_indices[
                    candidate_depths <= front_depth + self.depth_band
                ]
                if surface_indices.size >= self.min_lidar_points:
                    map_points = (
                        output_from_cloud @ homogeneous[surface_indices].T
                    ).T[:, :3]
                    lidar_position = np.median(map_points, axis=0)

            depth_position = None
            if calibration is not None:
                left = max(0, int(math.floor(x_min)))
                right = min(self.mask.shape[1], int(math.ceil(x_max)) + 1)
                top = max(0, int(math.floor(y_min)))
                bottom = min(self.mask.shape[0], int(math.ceil(y_max)) + 1)
                region_mask = self.mask[top:bottom, left:right] > 0
                rows, columns = np.nonzero(region_mask)
                if rows.size >= 5:
                    image_rows = rows + top
                    image_columns = columns + left
                    relative_value = float(
                        np.median(relative_depth[image_rows, image_columns])
                    )
                    metric_depth = self.metric_from_relative(relative_value, calibration)
                    if self.min_range <= metric_depth <= self.max_range:
                        target_u = float(np.median(image_columns))
                        target_v = float(np.median(image_rows))
                        camera_point = np.array(
                            [
                                metric_depth,
                                (self.cx - target_u) * metric_depth / self.fx,
                                (self.cy - target_v) * metric_depth / self.fy,
                                1.0,
                            ]
                        )
                        cloud_from_camera = np.linalg.inv(camera_from_cloud)
                        depth_position = (output_from_cloud @ cloud_from_camera @ camera_point)[:3]

            if lidar_position is not None and depth_position is not None:
                weight = self.depth_lidar_weight
                localized_points.append(weight * lidar_position + (1.0 - weight) * depth_position)
            elif depth_position is not None:
                localized_points.append(depth_position)
            elif lidar_position is not None:
                localized_points.append(lidar_position)

        self.publish_results(localized_points, self.box_stamp)

    def publish_results(self, points, stamp=None):
        stamp = rospy.Time.now() if stamp is None else stamp
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self.output_frame
        markers = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers.markers.append(clear_marker)

        for index, point in enumerate(points):
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.w = 1.0
            poses.poses.append(pose)

            marker = Marker()
            marker.header = poses.header
            marker.ns = "victim_detection"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.0
            marker.color.a = 0.9
            markers.markers.append(marker)

        self.pose_publisher.publish(poses)
        self.count_publisher.publish(UInt32(data=len(points)))
        self.marker_publisher.publish(markers)
        if points:
            rospy.loginfo_throttle(
                2.0, "Localized %d victim(s) in map frame", len(points)
            )


def main():
    rospy.init_node("victim_pointcloud_localizer")
    VictimPointCloudLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
