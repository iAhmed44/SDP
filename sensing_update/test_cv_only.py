#!/usr/bin/env python3
"""Webcam + object detection + pose only. No LiDAR, GPIO or NavBridge instances."""
import argparse
import os
from pathlib import Path
import signal
import threading
import time
from main_sensing import load_config
from sensing_logger import SensingLogger,LoggedLatest
from version import VERSION


def main():
    base=Path(__file__).resolve().parent
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--version',action='version',version='Sensing CV '+VERSION)
    p.add_argument('--config',default=str(base/'config.json'))
    p.add_argument('--models',default=str(base.parent/'cv'))
    p.add_argument('--no-display',action='store_true')
    p.add_argument('--duration',type=float,default=0)
    p.add_argument('--imgsz',type=int)
    p.add_argument('--log-mode',choices=['compact','detailed'],default='compact')
    args=p.parse_args();cfg=load_config(args.config)
    if args.imgsz:
        if not 160<=args.imgsz<=1280:p.error('--imgsz must be 160–1280')
        cfg['camera']['imgsz']=args.imgsz
    cfg['navigation']['enabled']=False;cfg['logging']['mode']=args.log_mode
    cfg['run_type']='CV_ONLY'
    if not args.no_display:
        os.environ.setdefault('QT_QPA_PLATFORM','xcb')
        fonts='/usr/share/fonts/truetype/dejavu'
        if Path(fonts).is_dir():os.environ['QT_QPA_FONTDIR']=fonts
    from cv.yolo_multiplexer import YOLOMultiplexer
    from ui import annotate_camera,cv2
    stop=threading.Event();logger=SensingLogger(base,cfg,motors=False)
    results=LoggedLatest(logger);camera=YOLOMultiplexer(cfg['camera'],args.models,stop,results)
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    start=time.monotonic();last_sequence=None
    print('[CV ONLY] OD + pose; no LiDAR or motor access. Q/Esc/Ctrl+C to stop.',flush=True)
    try:
        camera.start()
        while not stop.is_set():
            now=time.monotonic()
            if args.duration>0 and now-start>=args.duration:break
            v=results.get()
            if v and v.get('sequence')!=last_sequence:
                last_sequence=v.get('sequence')
                names=[d.get('display_label',d['name']) for d in v.get('detections',[])]
                print(f"[CV] {v.get('task')} {v.get('inference_ms',0):.1f}ms valid={v.get('valid')} detections={names} pose={v.get('pose_status')}",flush=True)
            if not args.no_display:
                frame=(v or {}).get('frame')
                if frame is not None:
                    image=annotate_camera(frame.copy(),v,cfg)
                    fresh=bool(v.get('valid') and now-v['timestamp']<=cfg['camera']['max_result_age_s'])
                    label=f"CV ONLY | {v.get('task')} | {v.get('inference_ms',0):.0f}ms | {'FRESH' if fresh else 'STALE'} | {v.get('pose_status')}"
                    cv2.putText(image,label,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,200,255),2)
                    cv2.imshow('CV ONLY - OD + pose',image)
                if cv2.waitKey(20)&255 in (ord('q'),ord('Q'),27):break
            else:stop.wait(.05)
    finally:
        stop.set();camera.close()
        if not args.no_display:cv2.destroyAllWindows()
        print('[CV ONLY] Stopped. No motor commands were issued.',flush=True)
        logger.close()


if __name__=='__main__':main()
