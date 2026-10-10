import os
import time
import cv2
import numpy as np
import threading

from ultralytics import YOLO


class YOLOMultiplexer:

    def __init__(
        self,
        det_model_name="yolo26n.pt",
        pose_model_name="yolo26n-pose.pt"
    ):

        # ========================================================
        # MODEL PATHS
        # ========================================================

        base_dir = os.path.dirname(
            os.path.abspath(__file__)
        )

        det_model_path = os.path.join(
            base_dir,
            det_model_name
        )

        pose_model_path = os.path.join(
            base_dir,
            pose_model_name
        )

        print(
            f"[YOLO] Loading detection model: "
            f"{det_model_path}"
        )

        self.det_model = YOLO(
            det_model_path
        )

        print(
            f"[YOLO] Loading pose model: "
            f"{pose_model_path}"
        )

        self.pose_model = YOLO(
            pose_model_path
        )


        # ========================================================
        # CAMERA SETTINGS
        # ========================================================

        self.cap = None

        self.frame_count = 0

        self.hfov_deg = 66.0

        self.frame_width = 640
        self.frame_height = 360

        self.camera_fps = 30

        self.yolo_imgsz = 416


        # ========================================================
        # CAMERA THREAD STATE
        # ========================================================

        self.capture_thread = None

        self.capture_running = False

        self.latest_frame = None

        self.frame_lock = threading.Lock()

        self.frame_ready = threading.Event()


        # ========================================================
        # OPTICAL FLOW SETTINGS
        # ========================================================

        self.prev_gray = None
        self.p0 = None

        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(
                cv2.TERM_CRITERIA_EPS
                |
                cv2.TERM_CRITERIA_COUNT,
                10,
                0.03
            )
        )

        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )


    # ============================================================
    # CAMERA STARTUP
    # ============================================================

    def start_camera(
        self,
        device_index=0
    ):

        print(
            f"[CAMERA] Opening /dev/video{device_index}..."
        )

        self.cap = cv2.VideoCapture(
            device_index,
            cv2.CAP_V4L2
        )

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera index {device_index}"
            )


        # --------------------------------------------------------
        # Force MJPEG
        # --------------------------------------------------------

        mjpg_fourcc = cv2.VideoWriter_fourcc(
            *"MJPG"
        )

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            mjpg_fourcc
        )


        # --------------------------------------------------------
        # Resolution
        # --------------------------------------------------------

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.frame_width
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.frame_height
        )


        # --------------------------------------------------------
        # FPS
        # --------------------------------------------------------

        self.cap.set(
            cv2.CAP_PROP_FPS,
            self.camera_fps
        )


        # --------------------------------------------------------
        # Minimize internal buffering
        # --------------------------------------------------------

        self.cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )


        # --------------------------------------------------------
        # Read actual camera settings
        # --------------------------------------------------------

        actual_width = int(
            self.cap.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            self.cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        actual_fps = self.cap.get(
            cv2.CAP_PROP_FPS
        )

        actual_fourcc = int(
            self.cap.get(
                cv2.CAP_PROP_FOURCC
            )
        )

        fourcc_string = "".join(
            [
                chr(
                    (
                        actual_fourcc
                        >>
                        (8 * i)
                    )
                    &
                    0xFF
                )
                for i in range(4)
            ]
        )

        print(
            "[CAMERA] Opened successfully."
        )

        print(
            f"[CAMERA] Resolution: "
            f"{actual_width}x{actual_height}"
        )

        print(
            f"[CAMERA] FPS: "
            f"{actual_fps}"
        )

        print(
            f"[CAMERA] Format: "
            f"{fourcc_string}"
        )


        if actual_width > 0:
            self.frame_width = actual_width

        if actual_height > 0:
            self.frame_height = actual_height


        # --------------------------------------------------------
        # Flush startup frames
        # --------------------------------------------------------

        for _ in range(5):
            self.cap.grab()


        # ========================================================
        # START CAPTURE THREAD
        # ========================================================

        self.capture_running = True

        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )

        self.capture_thread.start()

        print(
            "[CAMERA] Latest-frame capture thread started."
        )


        # Wait briefly for first frame
        if not self.frame_ready.wait(
            timeout=3.0
        ):

            raise RuntimeError(
                "Camera opened but no frame was received."
            )


    # ============================================================
    # CAMERA CAPTURE THREAD
    # ============================================================

    def _capture_loop(self):

        while self.capture_running:

            if (
                self.cap is None
                or
                not self.cap.isOpened()
            ):
                break


            ret, frame = self.cap.read()


            if not ret:
                time.sleep(0.005)
                continue


            # ----------------------------------------------------
            # Replace old frame with newest frame
            # ----------------------------------------------------

            with self.frame_lock:

                self.latest_frame = frame


            self.frame_ready.set()


    # ============================================================
    # GET LATEST FRAME
    # ============================================================

    def _get_latest_frame(self):

        with self.frame_lock:

            if self.latest_frame is None:
                return None


            # Copy so camera thread can immediately overwrite
            # the shared latest frame.
            return self.latest_frame.copy()


    # ============================================================
    # OPTICAL FLOW TILT TRACKING
    # ============================================================

    def _track_pitch_tilt(
        self,
        gray_frame
    ):

        tilt_alert = False


        if (
            self.prev_gray is None
            or
            self.p0 is None
            or
            len(self.p0) < 10
        ):

            self.p0 = cv2.goodFeaturesToTrack(
                gray_frame,
                mask=None,
                **self.feature_params
            )

            self.prev_gray = gray_frame

            return False


        p1, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray_frame,
            self.p0,
            None,
            **self.lk_params
        )


        if (
            p1 is not None
            and
            status is not None
        ):

            good_new = p1[
                status == 1
            ]

            good_old = self.p0[
                status == 1
            ]


            if len(good_new) > 0:

                dy_array = (
                    good_new[:, 1]
                    -
                    good_old[:, 1]
                )


                if np.median(
                    dy_array
                ) < -15.0:

                    tilt_alert = True


                self.p0 = good_new.reshape(
                    -1,
                    1,
                    2
                )


            else:

                self.p0 = cv2.goodFeaturesToTrack(
                    gray_frame,
                    mask=None,
                    **self.feature_params
                )


        else:

            self.p0 = cv2.goodFeaturesToTrack(
                gray_frame,
                mask=None,
                **self.feature_params
            )


        self.prev_gray = gray_frame

        return tilt_alert


    # ============================================================
    # PROCESS NEWEST FRAME
    # ============================================================

    def process_frame(self):

        if not self.frame_ready.is_set():

            return (
                "error",
                [],
                False,
                None
            )


        # ========================================================
        # GET ONLY THE NEWEST FRAME
        # ========================================================

        t0 = time.perf_counter()

        frame = self._get_latest_frame()

        t1 = time.perf_counter()


        if frame is None:

            return (
                "error",
                [],
                False,
                None
            )


        # --------------------------------------------------------
        # IMPORTANT:
        # Do not mirror the processing frame.
        #
        # Camera left/right must remain consistent with LiDAR.
        # --------------------------------------------------------


        # ========================================================
        # OPTICAL FLOW
        # ========================================================

        gray_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        tilt_alert = self._track_pitch_tilt(
            gray_frame
        )


        detections = []


        # ========================================================
        # YOLO TIMING
        # ========================================================

        t2 = time.perf_counter()


        if self.frame_count % 3 == 1:

            task = "pose"

            results = self.pose_model(
                frame,
                verbose=False,
                imgsz=self.yolo_imgsz
            )

        else:

            task = "detect"

            results = self.det_model(
                frame,
                verbose=False,
                imgsz=self.yolo_imgsz
            )


        t3 = time.perf_counter()


        latest_frame_ms = (
            t1 - t0
        ) * 1000.0

        yolo_ms = (
            t3 - t2
        ) * 1000.0


        print(
            f"[PERF] "
            f"LatestFrame: {latest_frame_ms:.1f} ms | "
            f"YOLO: {yolo_ms:.1f} ms | "
            f"Task: {task}"
        )


        self.frame_count += 1


        # ========================================================
        # PARSE YOLO RESULTS
        # ========================================================

        if (
            results
            and
            len(results) > 0
        ):

            result = results[0]

            boxes = result.boxes

            keypoints = (
                result.keypoints
                if hasattr(
                    result,
                    "keypoints"
                )
                else
                None
            )


            if boxes is not None:

                for i, box in enumerate(
                    boxes
                ):

                    xyxy = (
                        box.xyxy[0]
                        .cpu()
                        .numpy()
                    )


                    cls_id = int(
                        box.cls[0]
                        .cpu()
                        .item()
                    )


                    conf = float(
                        box.conf[0]
                        .cpu()
                        .item()
                    )


                    cls_name = result.names.get(
                        cls_id,
                        str(cls_id)
                    )


                    # =================================================
                    # IMAGE X -> CAMERA ANGLE
                    # =================================================

                    center_x = (
                        xyxy[0]
                        +
                        xyxy[2]
                    ) / 2.0


                    norm_x = (

                        center_x

                        -

                        (
                            self.frame_width
                            /
                            2.0
                        )

                    ) / (

                        self.frame_width
                        /
                        2.0
                    )


                    angle_deg = (

                        norm_x

                        *

                        (
                            self.hfov_deg
                            /
                            2.0
                        )
                    )


                    det_data = {

                        "cls_id":
                            cls_id,

                        "name":
                            cls_name,

                        "conf":
                            conf,

                        "bbox":
                            xyxy.tolist(),

                        "angle_deg":
                            float(
                                angle_deg
                            )
                    }


                    # =================================================
                    # POSE KEYPOINTS
                    # =================================================

                    if (
                        keypoints is not None
                        and
                        keypoints.xy is not None
                        and
                        len(keypoints.xy) > i
                    ):

                        kpts = (
                            keypoints.xy[i]
                            .cpu()
                            .numpy()
                        )


                        det_data[
                            "keypoints"
                        ] = kpts.tolist()


                    detections.append(
                        det_data
                    )


        return (
            task,
            detections,
            tilt_alert,
            frame
        )


    # ============================================================
    # RELEASE CAMERA
    # ============================================================

    def release(self):

        print(
            "[CAMERA] Shutting down..."
        )


        # Stop capture thread first
        self.capture_running = False


        if (
            self.capture_thread is not None
            and
            self.capture_thread.is_alive()
        ):

            self.capture_thread.join(
                timeout=1.0
            )


        if self.cap is not None:

            try:
                self.cap.release()

            except Exception:
                pass


        self.cap = None

        self.latest_frame = None

        self.prev_gray = None

        self.p0 = None

        self.frame_ready.clear()


        print(
            "[CAMERA] Camera released."
        )
