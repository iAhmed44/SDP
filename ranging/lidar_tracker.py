import time
import math
import serial
import numpy as np
from rplidar import RPLidar

class LidarTracker:
    def __init__(self, port='/dev/ttyUSB0', baud=460800):
        self.port = port
        self.baud = baud
        self.lidar = None
        self.v_user = 0.0
        self.tracked_clusters = {}  # id -> state dict
        self.next_cluster_id = 0
        self.last_timestamp = time.time()

    def connect_and_purge(self):
        """Establishes connection and purges the serial backlog."""
        self.lidar = RPLidar(self.port, baudrate=self.baud, timeout=3)
        try:
            self.lidar.stop()
            time.sleep(0.5)
            self.lidar.stop_motor()
            time.sleep(0.5)

            if hasattr(self.lidar, '_serial') and self.lidar._serial is not None:
                self.lidar._serial.reset_input_buffer()
                while self.lidar._serial.in_waiting > 0:
                    self.lidar._serial.read(self.lidar._serial.in_waiting)
                    time.sleep(0.05)

            self.lidar.start_motor()
            time.sleep(1.5)
        except Exception as e:
            if self.lidar:
                self.lidar.disconnect()
            raise RuntimeError(f"LiDAR initialization failure: {e}")

    def cluster_points(self, scan):
        """
        Filters raw points to the front 180° arc (-90° to +90°) and groups
        neighboring points into spatial clusters using adaptive Euclidean distance.
        """
        valid_points = []
        for quality, angle, dist_mm in scan:
            if dist_mm <= 100 or dist_mm > 8000:
                continue
            
            # Map [0, 360) to [-90, +90]
            rel_angle = None
            if 0 <= angle <= 90:
                rel_angle = angle
            elif 270 <= angle < 360:
                rel_angle = angle - 360

            if rel_angle is not None:
                dist_m = dist_mm / 1000.0
                valid_points.append((rel_angle, dist_m))

        if not valid_points:
            return []

        # Sort points by angle
        valid_points.sort(key=lambda p: p[0])

        clusters = []
        curr_cluster = [valid_points[0]]

        for i in range(1, len(valid_points)):
            prev_p = valid_points[i - 1]
            curr_p = valid_points[i]

            # Angular distance and radial separation
            d_angle = abs(curr_p[0] - prev_p[0])
            d_radial = abs(curr_p[1] - prev_p[1])

            # Points belong to the same object if closely spaced
            if d_angle < 4.0 and d_radial < 0.25:
                curr_cluster.append(curr_p)
            else:
                clusters.append(self._summarize_cluster(curr_cluster))
                curr_cluster = [curr_p]

        if curr_cluster:
            clusters.append(self._summarize_cluster(curr_cluster))

        return clusters

    def _summarize_cluster(self, points):
        angles = [p[0] for p in points]
        dists = [p[1] for p in points]
        return {
            "angle_deg": float(np.mean(angles)),
            "dist_m": float(np.min(dists)),
            "count": len(points)
        }

    def update_states(self, detected_clusters, cv_detections):
        """
        Tracks clusters across frames, performs LiDAR range-rate ego-motion
        inversion, and executes the dual-threshold static/dynamic state machine.
        """
        now = time.time()
        dt = max(now - self.last_timestamp, 0.05)
        self.last_timestamp = now

        # 1. Estimate Ego-Velocity via median range-rate inversion of forward objects
        v_candidates = []
        updated_tracked = {}

        for det in detected_clusters:
            # Associate with closest previous tracked cluster
            best_id = None
            min_dist_diff = 0.4  # Association gate in meters

            for c_id, prev in self.tracked_clusters.items():
                if abs(det["angle_deg"] - prev["angle_deg"]) < 10.0:
                    dist_diff = abs(det["dist_m"] - prev["dist_m"])
                    if dist_diff < min_dist_diff:
                        min_dist_diff = dist_diff
                        best_id = c_id

            if best_id is not None:
                prev = self.tracked_clusters[best_id]
                v_rel = (det["dist_m"] - prev["dist_m"]) / dt
                c_id = best_id
                stop_time = prev["stop_time"]
            else:
                v_rel = 0.0
                c_id = self.next_cluster_id
                self.next_cluster_id += 1
                stop_time = now

            # Collect candidates for user forward velocity
            if abs(det["angle_deg"]) < 30 and abs(v_rel) > 0.1:
                v_cand = -v_rel / math.cos(math.radians(det["angle_deg"]))
                if 0.3 <= v_cand <= 2.2:
                    v_candidates.append(v_cand)

            updated_tracked[c_id] = {
                "id": c_id,
                "angle_deg": det["angle_deg"],
                "dist_m": det["dist_m"],
                "v_rel": v_rel,
                "stop_time": stop_time,
                "last_seen": now
            }

        # Update ego-motion speed if valid stationary candidates exist
        if v_candidates:
            self.v_user = float(np.median(v_candidates))

        # 2. Semantic Association & Dual-Threshold Classification
        dynamic_threats = []
        static_obstacles = []

        for c_id, c in updated_tracked.items():
            # Compensate relative velocity with estimated user ego-motion
            v_abs = c["v_rel"] + self.v_user * math.cos(math.radians(c["angle_deg"]))

            # Match with YOLO detections within +/- 5 degrees
            semantic_class = "obstacle"
            for cv_obj in cv_detections:
                if abs(c["angle_deg"] - cv_obj["angle_deg"]) <= 5.0:
                    semantic_class = cv_obj["name"]
                    break

            c["semantic_class"] = semantic_class

            # State machine: 7.0s patience for humans, 2.0s for inanimate obstacles
            patience = 7.0 if semantic_class == "person" else 2.0

            if abs(v_abs) < 0.15:
                stationary_duration = now - c["stop_time"]
                if stationary_duration >= patience:
                    c["state"] = "STATIC"
                    static_obstacles.append(c)
                else:
                    c["state"] = "DYNAMIC"
                    dynamic_threats.append(c)
            else:
                c["stop_time"] = now
                c["state"] = "DYNAMIC"
                dynamic_threats.append(c)

        self.tracked_clusters = updated_tracked
        return dynamic_threats, static_obstacles

    def close(self):
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass