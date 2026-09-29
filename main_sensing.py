import sys
import time
from cv.yolo_multiplexer import YOLOMultiplexer
from ranging.lidar_tracker import LidarTracker
from haptics.motor_driver import MotorDriver
from comms.nav_bridge import NavBridge

def main():
    print("[INIT] Initializing Sensing Subsystem...")
    
    # 1. Hardware & Module Instantiation
    # CRITICAL UPDATE: Explicitly pass your baked-in local model names here
    vision = YOLOMultiplexer(det_model_name="yolo26n.pt", pose_model_name="yolo26n-pose.pt")
    vision.start_camera(device_index=0)

    lidar = LidarTracker(port='/dev/ttyUSB0', baud=460800)
    lidar.connect_and_purge()

    haptics = MotorDriver()
    nav_bridge = NavBridge(pub_port=5555)

    print("[RUNNING] Subsystem active. Real-time perception loop started.")

    try:
        # Stream scans directly from RPLidar
        for scan in lidar.lidar.iter_scans(scan_type='normal', max_buf_meas=500):
            # A. Process Camera (YOLO Detection / Pose + Optical Flow Tilt Tracking)
            task, cv_detections, tilt_alert, _ = vision.process_frame()

            # Virtual IMU Check: If user pitches forward abruptly, ignore this scan 
            # to prevent floor returns from registering as static walls.
            if tilt_alert:
                continue

            # B. Extract and cluster LiDAR data in the front 180° arc
            clusters = lidar.cluster_points(scan)

            # C. Fuse CV + LiDAR: compensate ego-motion and evaluate states
            # Uses range-rate inversion for walking speed and YOLO classes for patience thresholds.
            dynamic_threats, static_obstacles = lidar.update_states(clusters, cv_detections)

            # D. Actuation: route dynamic threats into the 5-motor haptic array
            haptics.apply_feedback(dynamic_threats)

            # E. Navigation Comms: stream verified static objects to the routing costmap
            if static_obstacles:
                nav_bridge.publish_static_map(static_obstacles, ego_velocity=lidar.v_user)

    except KeyboardInterrupt:
        print("\n[STOP] Shutting down Sensing Subsystem...")
    except Exception as e:
        print(f"\n[ERROR] Unhandled runtime fault: {e}")
    finally:
        haptics.stop_all()
        lidar.close()
        vision.release()
        nav_bridge.close()
        print("[EXIT] Clean hardware termination complete.")

if __name__ == '__main__':
    main()