"""Latest-frame V4L2 camera, separate inference worker, no mirror flip.
82.1 degrees is the approved 16:9 estimate, not a lens calibration.
Optical flow is only a confidence/tilt diagnostic, never a reason to stop haptics.
"""
import math
from pathlib import Path
import threading
import time
import cv2
import numpy as np
from core import Latest, pixel_angle
from cv.pose_support import attach_pose
from cv.display_labels import numbered_detections


class BackgroundMotion:
    def __init__(self):
        self.gray = None
        self.time = None
        self.boxes = []

    def reset(self):
        self.gray, self.time, self.boxes = None, None, []

    def update(self, frame, detections, timestamp):
        # 320px processing width limits the overhead from 720p capture.
        w,h = frame.shape[1], frame.shape[0]
        gray = cv2.cvtColor(cv2.resize(frame, (320, round(h*320/w))), cv2.COLOR_BGR2GRAY)
        valid, dx, dy = False, 0.0, 0.0
        dt = timestamp-self.time if self.time is not None else 0
        if self.gray is not None and 0.01<=dt<=0.5 and self.gray.shape==gray.shape:
            mask = np.full_like(self.gray, 255)
            for b in self.boxes:
                x1,y1,x2,y2 = [round(v*320/w) for v in b]
                cv2.rectangle(mask,(x1,y1),(x2,y2),0,-1)
            p0 = cv2.goodFeaturesToTrack(self.gray, mask=mask, maxCorners=100, qualityLevel=.02, minDistance=8)
            if p0 is not None and len(p0)>=12:
                p1,st,err = cv2.calcOpticalFlowPyrLK(self.gray,gray,p0,None)
                if p1 is not None:
                    good = (st.reshape(-1)==1)&(err.reshape(-1)<30)
                    if np.count_nonzero(good)>=12:
                        delta = (p1-p0).reshape(-1,2)[good]
                        dx,dy = map(float,np.median(delta,axis=0))
                        scatter = float(np.median(np.linalg.norm(delta-[dx,dy],axis=1)))
                        valid = scatter<3
        self.gray, self.time = gray,timestamp
        self.boxes = [d['bbox'] for d in detections]
        # Do not call this an IMU or assert true camera rotation from image flow.
        return {'valid':valid, 'dx_px_320':dx, 'dy_px_320':dy,
                'uncertain':not valid or abs(dx)>8 or abs(dy)>8,
                'tilt_alert':valid and abs(dy)>8}


class YOLOMultiplexer:
    def __init__(self, cfg, model_dir, stop, result_slot):
        self.cfg, self.model_dir, self.stop, self.results = cfg, Path(model_dir), stop, result_slot
        self.frames = Latest()
        self.cap_thread = None
        self.infer_thread = None
        self.status = Latest()
        self.status.set('Starting camera')
        self.actual = Latest()
        self.actual.set({})

    def start(self):
        self.cap_thread = threading.Thread(target=self._capture, daemon=True)
        self.infer_thread = threading.Thread(target=self._infer, daemon=True)
        self.cap_thread.start()
        self.infer_thread.start()

    def _capture(self):
        sequence = 0
        while not self.stop.is_set():
            cap = None
            try:
                cap = cv2.VideoCapture(self.cfg['device'],cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError('Cannot open USB webcam')
                cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.cfg['width'])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.cfg['height'])
                cap.set(cv2.CAP_PROP_FPS,self.cfg['fps'])
                cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
                self.actual.set({'width':int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                                 'height':int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                                 'fps_requested':self.cfg['fps'],'fps_reported':cap.get(cv2.CAP_PROP_FPS)})
                print('[CAMERA]',self.actual.get(),flush=True)
                while not self.stop.is_set():
                    ok,frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError('USB webcam read failed')
                    sequence+=1
                    self.frames.set({'frame':frame, 'timestamp':time.monotonic(), 'sequence':sequence})
                    self.status.set('Camera receiving')
            except Exception as exc:
                self.frames.set(None)
                self.results.set({'valid':False,'error':str(exc),'timestamp':time.monotonic()})
                self.status.set(str(exc)+'; retrying')
                print('[CAMERA]',exc,flush=True)
            finally:
                if cap is not None:
                    cap.release()
            self.stop.wait(1.0)

    def _infer(self):
        try:
            from ultralytics import YOLO
            path = self.model_dir/'yolo26n.pt'
            if not path.is_file():
                raise FileNotFoundError(f'Missing {path}; supply --models with your existing cv folder')
            model = YOLO(str(path))
            pose = None
            pose_status='disabled'
            if self.cfg['pose_every']>0:
                try:
                    pose_path = self.model_dir/'yolo26n-pose.pt'
                    if not pose_path.is_file():raise FileNotFoundError(str(pose_path))
                    pose = YOLO(str(pose_path))
                    pose_status='ready'
                except Exception as exc:
                    pose_status='unavailable: '+str(exc)
                    print('[POSE]',pose_status,'; OD remains active',flush=True)
            motion = BackgroundMotion()
            sequence, count = 0,0
            previous_finished=None
            stale_skipped=0
            cached_od, cached_poses = [],[]
            last_shape=None
            while not self.stop.is_set():
                packet = self.frames.get()
                if not packet or packet['sequence']==sequence:
                    self.stop.wait(.01)
                    continue
                sequence = packet['sequence']
                if time.monotonic()-packet['timestamp']>self.cfg['max_frame_age_s']:
                    motion.reset()
                    stale_skipped+=1
                    continue
                frame = packet['frame']
                width,height = frame.shape[1],frame.shape[0]
                if last_shape!=frame.shape:
                    cached_od,cached_poses=[],[]
                    motion.reset()
                    last_shape=frame.shape
                count+=1
                # One model call per cycle: OD, OD, POSE by default.
                task='pose' if pose is not None and count%self.cfg['pose_every']==0 else 'detect'
                selected=pose if task=='pose' else model
                start=time.monotonic()
                try:
                    result=selected(frame,verbose=False,imgsz=self.cfg['imgsz'],conf=self.cfg['confidence'],device='cpu')[0]
                except Exception as exc:
                    if task!='pose':raise
                    pose_status='failed: '+str(exc)
                    print('[POSE]',pose_status,'; continuing OD',flush=True)
                    pose=None;cached_poses=[]
                    continue
                detections=[]
                keypoints=result.keypoints if task=='pose' else None
                xy=keypoints.xy.cpu().tolist() if keypoints is not None else []
                kc=keypoints.conf.cpu().tolist() if keypoints is not None and keypoints.conf is not None else []
                for i,b in enumerate(result.boxes):
                    box=b.xyxy[0].cpu().tolist()
                    cls=int(b.cls[0].item())
                    d={'name':result.names[cls],'conf':float(b.conf[0].item()),'bbox':box,
                       'timestamp':packet['timestamp'],'source_task':task,
                       'angle_deg':pixel_angle((box[0]+box[2])/2,width,self.cfg['hfov_deg']),
                       'angle_min_deg':pixel_angle(box[0],width,self.cfg['hfov_deg']),
                       'angle_max_deg':pixel_angle(box[2],width,self.cfg['hfov_deg'])}
                    if task=='pose' and i<len(xy):
                        d.update(pose_keypoints=xy[i],pose_confidence=kc[i] if i<len(kc) else [0]*len(xy[i]),
                                 pose_timestamp=packet['timestamp'])
                    detections.append(d)
                inference_finished=time.monotonic()
                finished=inference_finished
                if task=='pose':
                    cached_poses=[d for d in detections if 'pose_keypoints' in d]
                    # Non-human objects retain their own original OD timestamps.
                    detections.extend(d for d in cached_od if d['name']!='person' and
                                      0<=finished-d['timestamp']<=self.cfg['max_result_age_s'])
                else:
                    cached_od=detections
                    detections=attach_pose(detections,cached_poses,finished,self.cfg['pose_max_age_s'],self.cfg['pose_iou'])
                flow=motion.update(frame,detections,packet['timestamp'])
                detections=numbered_detections(detections)
                finished=time.monotonic()
                live_frame=self.frames.get()
                fresh=(finished-packet['timestamp']<=self.cfg['max_result_age_s'] and live_frame is not None
                       and finished-live_frame['timestamp']<=self.cfg['max_frame_age_s'])
                self.results.set({'valid':fresh,'timestamp':packet['timestamp'],'finished':finished,
                                  'sequence':sequence,'frame':frame,'detections':detections,
                                  'flow':flow,'task':task,'pose_status':pose_status,
                                  'inference_ms':(inference_finished-start)*1000,
                                  'frame_to_result_ms':(finished-packet['timestamp'])*1000,
                                  'pre_inference_age_ms':(start-packet['timestamp'])*1000,
                                  'postprocess_ms':(finished-inference_finished)*1000,
                                  'result_interval_ms':None if previous_finished is None else (finished-previous_finished)*1000,
                                  'stale_frames_skipped_total':stale_skipped,
                                  'width':width,'height':height})
                previous_finished=finished
        except Exception as exc:
            self.results.set({'valid':False,'error':f'{type(exc).__name__}: {exc}','timestamp':time.monotonic()})
            print('[VISION ERROR]',repr(exc),flush=True)

    def close(self):
        # Shared stop is set by the owner; worker threads own their resources.
        for worker in (self.cap_thread,self.infer_thread):
            if worker:
                worker.join(timeout=1.0)
                if worker.is_alive():
                    print('[CAMERA] Worker still finishing after the stop request.',flush=True)
