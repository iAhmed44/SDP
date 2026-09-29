import json
import time
import zmq

class NavBridge:
    """
    Unidirectional ZeroMQ publisher broadcasting static perception map updates
    to the navigation subsystem without polling external IMU telemetry.
    """
    def __init__(self, pub_port=5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{pub_port}")

    def publish_static_map(self, static_clusters, ego_velocity=0.0):
        """
        Publishes verified stationary obstacles to the navigation costmap.
        """
        payload = {
            "timestamp": time.time(),
            "ego_velocity_mps": round(float(ego_velocity), 2),
            "static_obstacles": [
                {
                    "angle_deg": round(float(c["angle_deg"]), 1),
                    "dist_m": round(float(c["dist_m"]), 2),
                    "x_m": round(float(c["dist_m"] * np.sin(np.radians(c["angle_deg"]))), 2),
                    "y_m": round(float(c["dist_m"] * np.cos(np.radians(c["angle_deg"]))), 2),
                    "semantic_class": c.get("semantic_class", "obstacle")
                }
                for c in static_clusters
            ]
        }
        self.socket.send_string(f"NAV_STATIC {json.dumps(payload)}")

    def close(self):
        self.socket.close()
        self.context.term()