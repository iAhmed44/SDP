"""Camera + LiDAR diagnostic display; does not own any hardware."""
import math
import time
import cv2
import numpy as np
from cv.display_labels import numbered_detections
from version import VERSION

WINDOW='EE499 sensing update - CV + LiDAR'


def annotate_camera(frame,vision,cfg):
    for d in numbered_detections(vision.get('detections',[])):
        color=d['class_color_bgr']
        pose_age=time.monotonic()-d.get('pose_timestamp',-1e9)
        if 0<=pose_age<=cfg['camera']['pose_max_age_s']:
            pts=d.get('pose_keypoints',[])
            conf=d.get('pose_confidence',[])
            visible={i for i,(point,c) in enumerate(zip(pts,conf)) if c>=cfg['camera']['keypoint_confidence'] and point!=[0,0]}
            edges=[(0,1),(0,2),(1,3),(2,4),(5,6),(5,7),(7,9),(6,8),(8,10),(5,11),(6,12),(11,12),(11,13),(13,15),(12,14),(14,16)]
            for a,b in edges:
                if a in visible and b in visible:
                    cv2.line(frame,tuple(map(int,pts[a])),tuple(map(int,pts[b])),(255,190,40),2)
            for i in visible:cv2.circle(frame,tuple(map(int,pts[i])),3,(255,190,40),-1)
        x1,y1,x2,y2=map(int,d['bbox'])
        cv2.rectangle(frame,(x1,y1),(x2,y2),color,2)
        cv2.putText(frame,f"{d['display_label']} {d['angle_deg']:+.1f}deg",(x1,max(22,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.65,color,2)
    return frame


def render(state,vision,cfg,motors=False):
    image=np.full((860,1500,3),(22,26,32),np.uint8)
    def text(s,x,y,color=(215,220,230),size=.5):
        cv2.putText(image,str(s),(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)
    text('CV + LiDAR v'+VERSION+' | '+('MOTORS ENABLED' if motors else 'DRY RUN - motors OFF'),20,30,(0,220,255),.7)
    if vision and vision.get('frame') is not None:
        frame=vision['frame'].copy()
        frame=annotate_camera(frame,vision,cfg)
        frame=cv2.resize(frame,(760,428))
        image[50:478,15:775]=frame
        if not state.get('vision_healthy'):
            cv2.rectangle(image,(15,50),(775,84),(20,20,150),-1)
            text('STALE CAMERA IMAGE - NOT USED FOR FUSION',25,74,(255,255,255),.6)
    else:
        text('Waiting for camera / LiDAR-only mode',40,260,size=.7)
    text('HFOV '+str(cfg['camera']['hfov_deg'])+' deg | requested capture '+str(cfg['camera']['width'])+'x'+str(cfg['camera']['height']),20,503)
    text(f"LiDAR: {state.get('lidar_status','starting')} | age: {state.get('scan_age_s',-1):.2f}s | timing {'OK' if state.get('scan_timing_valid') else 'UNCERTAIN'}",20,532)
    text(f"Vision: {'fresh' if state.get('vision_healthy') else 'unavailable/stale -> LiDAR fallback'}",20,558)
    if vision and 'inference_ms' in vision:
        text(f"Actual {vision.get('width',0)}x{vision.get('height',0)} | {vision.get('task','detect')} {vision['inference_ms']:.0f}ms | age {state.get('vision_age_s',-1):.2f}s",20,584)
    text('Ego speed: '+(f"{state.get('ego_speed',0):+.2f}m/s (forward-only estimate)" if state.get('ego_valid') else 'UNKNOWN'),20,610)
    if vision and vision.get('flow',{}).get('tilt_alert'):
        text('CAMERA FLOW: abrupt vertical image motion; motion labels uncertain',20,630,(40,150,255),.43)
    text('Hazards: initial warning then 25% reminder',20,654)
    for i,t in enumerate(state.get('threats',[])[:4]):
        text(f"ID {t['id']} {t['semantic_class']} {t['angle_deg']:+.1f}deg {t['dist_m']:.2f}m: {t['reason']}",20,680+i*24,(60,190,255),.45)
    if not state.get('threats'):
        text('No selected warning (not a guarantee of a clear path)',20,680,size=.46)
    phases={t.get('haptic_phase','initial') for t in state.get('active_threats',[])}
    text(f"Feedback: {','.join(sorted(phases)) or 'off'} | full-burst suppressed: {state.get('suppressed_ids',[])}",20,790,(0,220,255),.45)
    if vision:text('Pose: '+vision.get('pose_status','waiting'),20,814,size=.43)
    text('Q/Esc quit | sensors: +x right, +y forward | 2D scan, not a room map',20,838,size=.49)
    center=(1110,373)
    extent=3.0
    scale=300/extent
    def xy(x,y):return (round(center[0]+x*scale),round(center[1]-y*scale))
    cv2.ellipse(image,center,(300,300),0,180,360,(30,43,34),-1)
    for r in (.5,1,1.5,2,2.5,3):
        cv2.circle(image,center,round(r*scale),(60,70,75),1)
        text(f'{r:.1f}m',center[0]+5,center[1]-round(r*scale)+15,size=.37)
    cv2.circle(image,center,round(cfg['haptics']['warning_distance_m']*scale),(0,170,255),2)
    corridor=cfg['haptics']['corridor_half_width_m']
    cv2.rectangle(image,xy(-corridor,cfg['haptics']['warning_distance_m']),xy(corridor,0),(0,150,200),1)
    half=cfg['camera']['hfov_deg']/2
    for a in (-half+cfg['camera']['yaw_deg'],half+cfg['camera']['yaw_deg']):
        cv2.line(image,xy(cfg['camera']['x_m'],cfg['camera']['y_m']),xy(3*math.sin(math.radians(a)),3*math.cos(math.radians(a))),(120,160,240),1)
    live=state.get('lidar_healthy',False)
    for p in state.get('points',[]):
        if p['dist_m']>extent:continue
        color=(65,75,85) if not live else ((230,140,80) if abs(p['angle_deg'])>90 else (90,200,90))
        cv2.circle(image,xy(p['x_m'],p['y_m']),2,color,-1)
    for t in state.get('tracks',[]):
        if not t['observed']:continue
        p=xy(t['x_m'],t['y_m'])
        color=(0,220,255) if t.get('cv_confirmed') else (180,130,230)
        cv2.circle(image,p,5,color,1)
        text(str(t['id']),p[0]+6,p[1]-4,color,.4)
    for t in state.get('threats',[]):
        a=math.radians(t['angle_deg']);p=xy(t['dist_m']*math.sin(a),t['dist_m']*math.cos(a))
        cv2.circle(image,p,8,(20,50,255),2)
    cv2.circle(image,center,6,(255,255,255),-1)
    cv2.arrowedLine(image,center,(center[0],center[1]-25),(255,255,255),2)
    text('FRONT 0 deg',1050,58)
    text('LEFT -90',800,395)
    text('RIGHT +90',1345,395)
    text('REAR +/-180',1040,697)
    text('Yellow: CV match | purple: unknown | red: warning',810,726,size=.44)
    text('Corridor + close fallback; mounting yaw is CONFIGURABLE',810,748,size=.43)
    for i,(label,duty) in enumerate(zip(['FL','L','C','R','FR'],state.get('duties',[0]*5))):
        x=860+i*100
        cv2.rectangle(image,(x,770),(x+60,810),(70,80,90),1)
        if duty:cv2.rectangle(image,(x,810-round(40*duty)),(x+60,810),(0,170,255),-1)
        text(f'{label} {duty*100:.0f}%',x,830,size=.45)
    return image


def show(state,vision,cfg,motors=False):
    cv2.imshow(WINDOW,render(state,vision,cfg,motors))
    return cv2.waitKey(30)&255 not in (ord('q'),ord('Q'),27)
