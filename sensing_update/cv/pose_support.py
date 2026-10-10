"""Timestamped pose-to-box association. Keypoints are evidence, not intent labels."""
import copy
import math


def iou(a,b):
    x1,y1=max(a[0],b[0]),max(a[1],b[1])
    x2,y2=min(a[2],b[2]),min(a[3],b[3])
    overlap=max(0,x2-x1)*max(0,y2-y1)
    area=lambda q:max(0,q[2]-q[0])*max(0,q[3]-q[1])
    return overlap/max(1e-9,area(a)+area(b)-overlap)


def attach_pose(detections,poses,now,max_age,min_iou):
    result=copy.deepcopy(detections)
    candidates=[]
    for i,d in enumerate(result):
        if d['name']!='person':continue
        for j,p in enumerate(poses):
            age=now-p['pose_timestamp']
            if 0<=age<=max_age:
                score=iou(d['bbox'],p['bbox'])
                if score>=min_iou:candidates.append((-score,i,j))
    used_d,used_p=set(),set()
    for _,i,j in sorted(candidates):
        if i in used_d or j in used_p:continue
        for key in ('pose_keypoints','pose_confidence','pose_timestamp'):
            result[i][key]=copy.deepcopy(poses[j][key])
        used_d.add(i);used_p.add(j)
    return result


def pose_evidence(detection,now,cfg):
    if detection.get('name')!='person' or detection.get('pose_timestamp') is None:return None
    age=now-detection['pose_timestamp']
    if not 0<=age<=cfg['pose_max_age_s']:return None
    points=detection.get('pose_keypoints',[])
    confidence=detection.get('pose_confidence',[])
    visible=[i for i,(p,c) in enumerate(zip(points,confidence)) if len(p)==2 and
             all(math.isfinite(v) for v in p) and c>=cfg['keypoint_confidence'] and p!=[0,0]]
    return {'timestamp':detection['pose_timestamp'],'keypoints':points,'confidence':confidence,
            'visible_keypoints':len(visible),'torso_visible':all(i in visible for i in (5,6,11,12)),
            'facing':'UNKNOWN','intent':'UNKNOWN'}
