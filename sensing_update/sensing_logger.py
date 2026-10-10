"""Bounded asynchronous logging, checkpointed groups, optional raw detail.
Motor records describe commands, not measured physical vibration.
"""
import atexit
from collections import Counter
import gzip
import io
import shutil
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import select
import sys
import threading
import time
from version import VERSION


def clean(value):
    if isinstance(value, dict):
        return {str(k):clean(v) for k,v in value.items() if k!='frame'}
    if isinstance(value, (list,tuple)):return [clean(v) for v in value]
    if isinstance(value, float):return value if math.isfinite(value) else None
    if value is None or isinstance(value,(str,int,bool)):return value
    if hasattr(value,'item'):return clean(value.item())
    return str(value)


class LoggedLatest:
    """Latest-result slot that records EVERY published CV result."""
    def __init__(self,logger):
        self.logger=logger
        self.lock=threading.Lock()
        self.value=None
    def set(self,value):
        with self.lock:self.value=value
        self.logger.record_cv(value)
    def get(self):
        with self.lock:return self.value


class SensingLogger:
    SECTIONS=('CV','LIDAR','HAPTICS','TERMINAL')
    def __init__(self,base,cfg,motors=False,block_seconds=10,capture_terminal=True):
        self.cfg,self.motors=cfg,motors
        self.options=cfg.get('logging',{})
        self.detailed=self.options.get('mode','compact')=='detailed'
        self.period=1/self.options.get('snapshot_hz',2)
        self.block_seconds=block_seconds
        self.started=time.monotonic()
        self.wall_start=datetime.now().astimezone()
        folder=Path(base)/'Sensing_LOG';folder.mkdir(parents=True,exist_ok=True)
        self.stem=folder/('Sensing_LOG_'+self.wall_start.strftime('%Y%m%d_%H%M%S_%f'))
        self.path=self.stem.with_suffix('.log')
        self.pending=self.stem.with_suffix('.pending')
        self.paths=[self.path]
        self.header=('Sensing_LOG | '+self.wall_start.isoformat()+'\n'+
            'Software version: '+VERSION+'\n'+
            'Mode: '+('GPIO ENABLED' if motors else 'DRY RUN')+'; commanded PWM, not measured vibration.\n'+
            'Logging: '+('DETAILED' if self.detailed else 'COMPACT: CV metadata, 2 Hz sensor/PWM snapshots, immediate warning/health transitions')+'\n'+
            'A .pending checkpoint contains the current incomplete group during a run.\n'+
            'Configuration: '+json.dumps(clean(cfg),allow_nan=False)+'\n')
        self.file=self.path.open('x',encoding='utf-8')
        self.file.write(self.header);self.file.flush()
        self.events=queue.Queue(maxsize=self.options.get('queue_size',2048))
        self.lock=threading.Lock();self.stats=Counter()
        self.closed=False;self.failed=False
        self.last_scan=None;self.last_lidar=-float('inf');self.last_haptic=-float('inf')
        self.previous_cycle=None;self.cycle_count=0;self.cycle_max_gap=0.;self.cycle_unhealthy=0
        self.last_health=None;self.last_active=set();self.last_suppressed=set()
        self.original_out=None;self.original_err=None;self.pipe=None
        self.capture_stop=threading.Event();self.capture_thread=None
        self.writer=threading.Thread(target=self._write,name='SensingLogWriter',daemon=True)
        self.writer.start()
        if capture_terminal:
            try:self._capture_terminal()
            except Exception as exc:self.record_terminal('[LOG] FD capture unavailable: '+str(exc))
        atexit.register(self.close)
        print('[LOG] Saving to',self.path,flush=True)

    def _put(self,section,value,now=None,critical=False):
        now=time.monotonic() if now is None else now
        event=(max(0,now-self.started),datetime.now().astimezone().isoformat(timespec='milliseconds'),section,clean(value))
        with self.lock:
            if self.closed or self.failed:return
            self.stats['submitted_'+section]+=1
            try:self.events.put_nowait(event)
            except queue.Full:
                if critical:
                    try:self.events.get_nowait();self.stats['overload_evicted']+=1
                    except queue.Empty:pass
                    try:self.events.put_nowait(event);return
                    except queue.Full:pass
                self.stats['overload_dropped']+=1

    def record_cv(self,vision,now=None):
        if vision is None:return
        if self.detailed:self._put('CV',vision,now);return
        data={k:v for k,v in vision.items() if k not in ('frame','detections')}
        data['detections']=[]
        for d in vision.get('detections',[]):
            item={k:v for k,v in d.items() if k not in ('pose_keypoints','pose_confidence')}
            if 'pose_keypoints' in d:
                item['visible_keypoints']=sum(c>=self.cfg['camera']['keypoint_confidence'] for c in d.get('pose_confidence',[]))
            data['detections'].append(item)
        self._put('CV',data,now)

    def record_cycle(self,state,packet=None,now=None):
        now=time.monotonic() if now is None else now
        if self.previous_cycle is not None:
            self.cycle_max_gap=max(self.cycle_max_gap,now-self.previous_cycle)
        self.previous_cycle=now
        self.cycle_count+=1
        self.cycle_unhealthy+=int(not state.get('vision_healthy',False))
        health=(state.get('lidar_healthy'),state.get('vision_healthy'),state.get('scan_timing_valid'))
        if self.detailed or now-self.last_lidar>=self.period or health!=self.last_health:
            data={k:state.get(k) for k in ('lidar_healthy','vision_healthy','lidar_status','scan_age_s',
                                         'vision_age_s','ego_speed','ego_valid','scan_timing_valid')}
            data['control_cycles_since_record']=self.cycle_count
            data['max_control_gap_ms']=self.cycle_max_gap*1000
            data['vision_unhealthy_cycles_since_record']=self.cycle_unhealthy
            self.cycle_count=0;self.cycle_max_gap=0.;self.cycle_unhealthy=0
            fields=('id','observed','semantic_class','angle_deg','dist_m','x_m','y_m','motion_state',
                    'cv_confirmed','relative_motion','velocity_valid','geometry_stable')
            data['tracks']=[({k:v for k,v in t.items() if k not in ('points','cv_points')} if self.detailed
                            else {k:t.get(k) for k in fields}) for t in state.get('tracks',[])]
            if packet:
                data.update(scan_sequence=packet.get('sequence'),generation=packet.get('generation'),
                            scan_timestamp=packet.get('timestamp'),scan_duration=packet.get('scan_duration'))
                key=(packet.get('generation'),packet.get('sequence'))
                if self.detailed and key!=self.last_scan:
                    data['raw_completed_scan_q_angle_distance_m']=packet.get('points',[])
                    self.last_scan=key
            self._put('LIDAR',data,now,critical=health!=self.last_health)
            self.last_lidar=now;self.last_health=health
        active=state.get('active_threats',[])
        ids={t['encounter_id'] for t in active}
        suppressed=set(state.get('suppressed_ids',[]))
        changed=ids!=self.last_active
        encounter_events=state.get('encounter_events',[])
        if self.detailed or changed or encounter_events or now-self.last_haptic>=self.period or bool(suppressed)!=bool(self.last_suppressed):
            self._put('HAPTICS',{'gpio_enabled':self.motors,'event':'transition' if changed else 'snapshot',
                'started_encounters':sorted(ids-self.last_active),'ended_encounters':sorted(self.last_active-ids),
                'duties_fl_l_c_r_fr':state.get('duties',[0]*5),
                'selected_hazards':state.get('threats',[]),'active_bursts':active,
                'suppressed_ids':sorted(suppressed),
                'encounter_by_track':state.get('encounter_by_track',{}),
                'encounter_events':encounter_events},now,critical=changed or bool(encounter_events))
            self.last_haptic=now
        self.last_active=ids;self.last_suppressed=suppressed

    def record_shutdown(self,gpio_enabled,success,reason):
        self._put('HAPTICS',{'event':'shutdown','gpio_enabled':gpio_enabled,
            'zero_command_success':bool(success),'duties_fl_l_c_r_fr':[0]*5 if success else None,
            'reason':reason,'physical_output_verified':False},critical=True)

    def record_terminal(self,line,now=None):
        # Structured sections already contain routine object/strength records.
        if not self.detailed and line.startswith(('[OBJECT]','[HAPTICS] Already','[WARNING] ID=')):
            with self.lock:self.stats['routine_terminal_omitted']+=1
            return
        self._put('TERMINAL',{'line':line},now,critical=('ERROR' in line or 'CLEANUP' in line))

    def _capture_terminal(self):
        sys.stdout.flush();sys.stderr.flush()
        self.original_out=os.dup(1);self.original_err=os.dup(2)
        read_fd,write_fd=os.pipe()
        self.pipe=read_fd
        try:
            os.dup2(write_fd,1);os.dup2(write_fd,2)
        finally:os.close(write_fd)
        self.capture_thread=threading.Thread(target=self._read_terminal,name='SensingLogTerminal',daemon=True)
        self.capture_thread.start()

    def _read_terminal(self):
        pending=b''
        try:
            while True:
                readable,_,_=select.select([self.pipe],[],[],.1)
                if not readable:
                    if self.capture_stop.is_set():break
                    continue
                chunk=os.read(self.pipe,65536)
                if not chunk:break
                # Retain the normal terminal display, including native/child output.
                try:os.write(self.original_out,chunk)
                except OSError:pass
                pending+=chunk
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1)
                    self.record_terminal(line.decode('utf-8','replace'))
        except (OSError,ValueError):pass
        finally:
            if pending:self.record_terminal(pending.decode('utf-8','replace'))

    def _block_text(self,index,records,end,partial=False):
        begin=index*self.block_seconds;end=max(begin,min((index+1)*self.block_seconds,end))
        out=io.StringIO()
        out.write('\n'+'='*90+'\n')
        out.write(f'GROUP {index+1:04d} | elapsed {begin:.3f}s to {end:.3f}s'+(' | PARTIAL CHECKPOINT' if partial else '')+'\n')
        with self.lock:stats=dict(self.stats)
        out.write('Cumulative logging counters: '+json.dumps(stats,sort_keys=True)+'\n')
        def compact(v):
            if isinstance(v,float):return round(v,4)
            if isinstance(v,dict):return {k:compact(x) for k,x in v.items()}
            if isinstance(v,list):return [compact(x) for x in v]
            return v
        for section in self.SECTIONS:
            entries=records[section];out.write(f'\n--- {section} | {len(entries)} records ---\n')
            for elapsed,wall,data in entries:
                content=data['line'] if section=='TERMINAL' else json.dumps(data if self.detailed else compact(data),allow_nan=False,separators=(',',':'))
                out.write(f'[{wall} | +{elapsed:.3f}s] {content}\n')
        return out.getvalue()

    def _checkpoint(self,index,records,end):
        temporary=self.pending.with_suffix('.pending.tmp')
        with temporary.open('w',encoding='utf-8') as f:
            f.write(self.header);f.write(self._block_text(index,records,end,True))
            f.flush();os.fsync(f.fileno())
        os.replace(temporary,self.pending)

    def _finish_block(self,index,records,end,partial=False):
        self.file.write(self._block_text(index,records,end,partial))
        self.file.flush();os.fsync(self.file.fileno())
        if self.file.tell()>=self.options.get('max_part_mb',25)*1024*1024:
            self.file.close()
            self.path=Path(str(self.stem)+f'_part{len(self.paths)+1:03d}.log')
            self.paths.append(self.path)
            self.file=self.path.open('x',encoding='utf-8');self.file.write(self.header)

    def _write(self):
        index=0;records={s:[] for s in self.SECTIONS};checkpoint=time.monotonic()
        end=0;block_count=0
        try:
            while True:
                try:event=self.events.get(timeout=.1)
                except queue.Empty:event='tick'
                if event is None:
                    end=max(end,time.monotonic()-self.started)
                    self._finish_block(index,records,end,True)
                    self.file.write('\nEND OF RUN | '+datetime.now().astimezone().isoformat()+'\n')
                    self.file.flush();os.fsync(self.file.fileno());self.file.close()
                    self.pending.unlink(missing_ok=True)
                    if self.options.get('compress_on_close',True):
                        for path in self.paths:
                            target=Path(str(path)+'.gz');temporary=Path(str(target)+'.tmp')
                            with path.open('rb') as src,gzip.open(temporary,'wb',compresslevel=3) as dst:shutil.copyfileobj(src,dst)
                            with temporary.open('rb') as f:os.fsync(f.fileno())
                            os.replace(temporary,target);path.unlink()
                        self.path=Path(str(self.path)+'.gz')
                    break
                if event=='tick':elapsed=time.monotonic()-self.started
                else:elapsed,wall,section,data=event
                end=max(end,elapsed)
                target=int(elapsed/self.block_seconds)
                while target>index:
                    self._finish_block(index,records,elapsed)
                    index+=1;records={s:[] for s in self.SECTIONS};block_count=0
                if event!='tick':
                    if target<index:data=dict(data,late_receipt=True)
                    if block_count<10000:
                        records[section].append((elapsed,wall,data));block_count+=1
                    else:
                        with self.lock:self.stats['block_limit_dropped']+=1
                if time.monotonic()-checkpoint>=self.options.get('checkpoint_s',1):
                    self._checkpoint(index,records,end);checkpoint=time.monotonic()
        except Exception as exc:
            self.failed=True
            message=f'[LOG ERROR] File logging stopped: {exc}\n'.encode('utf-8','replace')
            try:os.write(self.original_err if self.original_err is not None else 2,message)
            except OSError:pass
        finally:
            if not self.file.closed:self.file.close()

    def close(self):
        if self.closed:return
        if self.original_out is not None:
            try:sys.stdout.flush();sys.stderr.flush()
            except Exception:pass
            os.dup2(self.original_out,1);os.dup2(self.original_err,2)
            self.capture_stop.set()
            if self.capture_thread:self.capture_thread.join(timeout=2)
        with self.lock:
            if self.closed:return
            self.closed=True
        if self.writer.is_alive():
            try:self.events.put(None,timeout=2)
            except queue.Full:
                self.failed=True
                os.write(2,b'[LOG ERROR] Shutdown queue full; checkpoint may be incomplete.\n')
        self.writer.join(timeout=8)
        if self.writer.is_alive():os.write(2,b'[LOG ERROR] Writer still draining; retain .log and .pending files.\n')
        elif not self.failed:print('[LOG] Finished:',self.path,flush=True)
        for fd in (self.pipe,self.original_out,self.original_err):
            if fd is not None:
                try:os.close(fd)
                except OSError:pass
        atexit.unregister(self.close)
