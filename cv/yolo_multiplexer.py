import os
import cv2
import numpy as np
from ultralytics import YOLO

class YOLOMultiplexer:
    def __init__(self, det_model_name="yolo26n.pt", pose_model_name="yolo26n-pose.pt"):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.det_model = YOLO(os.path.join(base_dir, det_model_name))
        self.pose_model = YOLO(os.path.join(base_dir, pose_model_name))
        
        self.cap = None
        self.frame_count = 0
        self.hfov_deg = 66.0
        self.frame_width = 1280
        self.frame_height = 720

        self.prev_gray = None
        self.p0 = None
        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    def start_camera(self, device_index=0):
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

    def _track_pitch_tilt(self, gray_frame):
        tilt_alert = False
        if self.prev_gray is None or self.p0 is None or len(self.p0) < 10:
            self.p0 = cv2.goodFeaturesToTrack(gray_frame, mask=None, **self.feature_params)
            self.prev_gray = gray_frame
            return False



        # Lucas-Kanade sparse optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray_frame, self.p0, None, **self.lk_params)
        
        if p1 is not None and st is not None:
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]
            
            if len(good_new) > 0:
                dy_array = good_new[:, 1] - good_old[:, 1]
                if np.median(dy_array) < -15.0:
                    tilt_alert = True
                self.p0 = good_new.reshape(-1, 1, 2)
            else:
                self.p0 = cv2.goodFeaturesToTrack(gray_frame, mask=None, **self.feature_params)
        else:
            self.p0 = cv2.goodFeaturesToTrack(gray_frame, mask=None, **self.feature_params)
            
        self.prev_gray = gray_frame
        return tilt_alert

    def process_frame(self):
        if self.cap is None or not self.cap.isOpened():
            return "error", [], False, None

        ret, frame = self.cap.read()
        if not ret:
            return "error", [], False, None

        frame = cv2.flip(frame, 1)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tilt_alert = self._track_pitch_tilt(gray_frame)

        detections = []
        if self.frame_count % 3 == 1:
            task = "pose"
            results = self.pose_model(frame, verbose=False, imgsz=640)
        else:
            task = "detect"
            results = self.det_model(frame, verbose=False, imgsz=640)

        self.frame_count += 1

        if results and len(results) > 0:
            boxes = results[0].boxes
            keypoints = results[0].keypoints if hasattr(results[0], 'keypoints') else None

            if boxes is not None:
                for i, box in enumerate(boxes):
                    xyxy = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())
                    cls_name = results[0].names.get(cls_id, str(cls_id))

                    center_x = (xyxy[0] + xyxy[2]) / 2.0
                    norm_x = (center_x - (self.frame_width / 2.0)) / (self.frame_width / 2.0)
                    angle_deg = norm_x * (self.hfov_deg / 2.0)

                    det_data = {
                        "cls_id": cls_id,
                        "name": cls_name,
                        "conf": conf,
                        "bbox": xyxy.tolist(),
                        "angle_deg": float(angle_deg)
                    }

                    # Extract skeletal keypoints if running the pose task
                    if keypoints is not None and keypoints.xy is not None and len(keypoints.xy) > i:
                        kpts = keypoints.xy[i].cpu().numpy()
                        det_data["keypoints"] = kpts.tolist()

                    detections.append(det_data)

        return task, detections, tilt_alert, frame

    def release(self):
        if self.cap is not None:
            self.cap.release()