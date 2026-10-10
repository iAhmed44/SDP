"""Non-blocking latest-state handoff; the network worker owns its ZMQ socket.
PUB/SUB is lossy. Every message replaces the previous body-frame snapshot.
"""
import json
import math
import threading
import time
import uuid


def payload(tracks, valid, ego_speed, ego_valid, ttl, source_timestamp=None):
    obstacles=[]
    if valid:
        for t in tracks:
            if t['observed'] and t['motion_state']=='STATIC':
                obstacles.append({k:t[k] for k in ('id','angle_deg','dist_m','x_m','y_m','semantic_class')})
    return {'schema_version':3,'timestamp':time.time(),'source_monotonic_s':source_timestamp,
            'frame':'body_x_right_y_forward','snapshot':True,'valid':bool(valid),'ttl_s':ttl,
            'ego_velocity_mps':ego_speed,'ego_velocity_valid':bool(ego_valid),'static_obstacles':obstacles}


class NavBridge:
    def __init__(self,cfg):
        self.cfg=cfg
        self.session_id=str(uuid.uuid4())
        self.sequence=0
        self.lock=threading.Lock()
        self.latest=None
        self.status='starting'
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,name='NavBridge',daemon=True)
        self.thread.start()

    def publish(self,tracks,valid,ego_speed,ego_valid,source_timestamp):
        # Small immutable numeric snapshot only. No JSON, disk or socket calls
        # on the control thread; replacing the slot cannot grow a backlog.
        data=payload(tracks,valid,ego_speed,ego_valid,self.cfg['ttl_s'],source_timestamp)
        with self.lock:self.latest=data

    def _message(self,shutdown=False):
        with self.lock:data=dict(self.latest) if self.latest else payload([],False,0,False,self.cfg['ttl_s'])
        now=time.monotonic()
        source=data['source_monotonic_s']
        age=now-source if source is not None else None
        valid=bool(data['valid'] and not shutdown and age is not None and
                   0<=age<=self.cfg.get('max_source_age_s',.4))
        self.sequence+=1
        data.update(session_id=self.session_id,sequence=self.sequence,timestamp=time.time(),
                    source_age_s=age,valid=valid,
                    status='shutdown' if shutdown else ('ok' if valid else 'source_unavailable_or_stale'))
        if not valid:
            data.update(static_obstacles=[],ego_velocity_valid=False)
        return 'NAV_STATIC '+json.dumps(data,allow_nan=False,separators=(',',':'))

    def _run(self):
        # Missing pyzmq, occupied port, or a disconnected subscriber must not
        # terminate perception/haptics. Retry bridge errors independently.
        while not self.stop.is_set():
            context=socket=None
            try:
                import zmq
                context=zmq.Context()
                socket=context.socket(zmq.PUB)
                socket.setsockopt(zmq.LINGER,0)
                socket.setsockopt(zmq.SNDHWM,2)
                socket.bind(self.cfg['bind'])
                self.status='publishing'
                print('[NAV] Publishing NAV_STATIC at',self.cfg['bind'],flush=True)
                while not self.stop.is_set():
                    try:socket.send_string(self._message(),flags=zmq.NOBLOCK)
                    except zmq.Again:pass
                    self.stop.wait(1/self.cfg.get('publish_hz',4))
                try:socket.send_string(self._message(shutdown=True),flags=zmq.NOBLOCK)
                except zmq.Again:pass
            except Exception as exc:
                self.status='unavailable: '+str(exc)
                print('[NAV ERROR]',self.status,'; sensing continues',flush=True)
                self.stop.wait(2)
            finally:
                if socket is not None:socket.close(linger=0)
                if context is not None:context.term()
        self.status='stopped'

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        if self.thread.is_alive():print('[NAV] Worker still stopping; receiver TTL must expire.',flush=True)


class SnapshotReceiver:
    """Example receiver policy for the same-host bridge; no navigation planner.
    Consumers must also expire their own costmap entries, even without messages.
    """
    def __init__(self,max_ttl=.75,same_host=True):
        self.same_host=same_host
        self.max_ttl=max_ttl
        self.session=None
        self.retired=set()
        self.sequence=-1
        self.deadline=0
        self.data=None

    def accept(self,message,now=None):
        now=time.monotonic() if now is None else now
        topic,body=message.split(' ',1)
        data=json.loads(body,parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
        if (topic!='NAV_STATIC' or data.get('schema_version')!=3 or
            data.get('frame')!='body_x_right_y_forward' or data.get('snapshot') is not True):
            raise ValueError('Unsupported navigation schema/topic/frame')
        session,seq=data.get('session_id'),data.get('sequence')
        ttl=data.get('ttl_s')
        if not isinstance(session,str) or not session or type(seq) is not int or seq<0:
            raise ValueError('Invalid session/sequence')
        if type(ttl) not in (int,float) or not math.isfinite(ttl) or not 0<ttl<=self.max_ttl:
            raise ValueError('Invalid TTL')
        if type(data.get('valid')) is not bool or not isinstance(data.get('static_obstacles'),list):
            raise ValueError('Invalid snapshot')
        if data['valid']:
            age=data.get('source_age_s')
            if type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<=.4:
                raise ValueError('Invalid source age')
            if self.same_host:
                source=data.get('source_monotonic_s')
                if type(source) not in (int,float) or not math.isfinite(source):
                    raise ValueError('Missing same-host source time')
                if not 0<=now-source<=.4:
                    data.update(valid=False,status='source_expired_in_transit',static_obstacles=[])
        for o in data['static_obstacles']:
            if not isinstance(o,dict) or not isinstance(o.get('semantic_class'),str) or type(o.get('id')) is not int:
                raise ValueError('Invalid obstacle')
            for k in ('x_m','y_m','angle_deg','dist_m'):
                if type(o.get(k)) not in (int,float) or not math.isfinite(o[k]):
                    raise ValueError('Invalid obstacle coordinate')
            if o['dist_m']<0 or not -90<=o['angle_deg']<=90:
                raise ValueError('Invalid range/angle')
        if session in self.retired or (session==self.session and seq<=self.sequence):
            return False
        if session!=self.session:
            if self.session:self.retired.add(self.session)
            self.session=session
        self.sequence=seq
        if not data['valid']:data['static_obstacles']=[]
        self.data=data
        self.deadline=now+ttl
        if data['valid'] and self.same_host:
            self.deadline=min(self.deadline,data['source_monotonic_s']+.4)
        return True

    def current(self,now=None):
        now=time.monotonic() if now is None else now
        if self.data is None or now>=self.deadline:
            return {'valid':False,'status':'expired_or_not_received','static_obstacles':[]}
        return self.data
