import cv2
from cv.yolo_multiplexer import YOLOMultiplexer

def main():
    vision = YOLOMultiplexer(det_model_name="yolo26n.pt", pose_model_name="yolo26n-pose.pt")
    
    print("[INIT] Starting Mac webcam...")
    vision.start_camera(device_index=0)

    # Standard COCO 17-keypoint skeletal connections
    skeleton_edges = [
        (0, 1), (0, 2), (1, 3), (2, 4),         # Head
        (5, 6), (5, 11), (6, 12), (11, 12),     # Torso
        (5, 7), (7, 9),                         # Left arm
        (6, 8), (8, 10),                        # Right arm
        (11, 13), (13, 15),                     # Left leg
        (12, 14), (14, 16)                      # Right leg
    ]

    try:
        while True:
            task, detections, tilt_alert, frame = vision.process_frame()
            
            if frame is None:
                break

            if tilt_alert:
                print("⚠️ TILT ALERT: Abrupt downward pitch detected!")
            
            for det in detections:
                x1, y1, x2, y2 = map(int, det["bbox"])
                color = (0, 255, 0) if task == "detect" else (255, 0, 0)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{det['name']} | {task.upper()} | Angle: {det['angle_deg']:.1f}", 
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                if "keypoints" in det:
                    kpts = det["keypoints"]
                    
                    # 1. Draw the connecting skeleton lines
                    for edge in skeleton_edges:
                        pt1_idx, pt2_idx = edge
                        if pt1_idx < len(kpts) and pt2_idx < len(kpts):
                            x_a, y_a = int(kpts[pt1_idx][0]), int(kpts[pt1_idx][1])
                            x_b, y_b = int(kpts[pt2_idx][0]), int(kpts[pt2_idx][1])
                            
                            # Only draw if both points have valid confident coordinates (> 0)
                            if x_a > 0 and y_a > 0 and x_b > 0 and y_b > 0:
                                cv2.line(frame, (x_a, y_a), (x_b, y_b), (255, 0, 255), 2)

                    # 2. Draw the joint dots on top
                    for kpt in kpts:
                        kx, ky = int(kpt[0]), int(kpt[1])
                        if kx > 0 and ky > 0:
                            cv2.circle(frame, (kx, ky), 4, (0, 255, 255), -1)

            cv2.imshow("Mac WebCam Test (Press 'q' to quit)", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        vision.release()
        cv2.destroyAllWindows()
        print("[EXIT] Camera closed.")

if __name__ == "__main__":
    main()