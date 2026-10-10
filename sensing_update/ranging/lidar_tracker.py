"""C1 acquisition in a supervised child process, protecting the warning loop.
The driver queue is bounded so its serial coroutine must yield to the consumer.
Scan boundaries are handled before quality/range filtering, including empty scans.
"""
import asyncio
import logging
import math
import multiprocessing as mp
import queue
import time


def latest_put(out, value):
    try:
        out.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        out.get(timeout=0.01)
    except queue.Empty:
        pass
    try:
        out.put_nowait(value)
    except queue.Full:
        pass


class Revolution:
    def __init__(self, max_duration, min_samples, min_duration=.05):
        self.previous = None
        self.started = None
        self.synchronized = False
        self.points = {}
        self.samples = 0
        self.sequence = 0
        self.min_duration = min_duration
        self.max_duration = max_duration
        self.min_samples = min_samples

    def feed(self, point, now):
        try:
            a = float(point['a_deg']) % 360
            if not math.isfinite(a):
                return None
        except (KeyError, ValueError, TypeError):
            return None
        boundary = self.previous is not None and self.previous > 300 and a < 60
        packet = None
        if boundary:
            if (self.synchronized and self.started is not None and
                    now-self.started <= self.max_duration and self.samples >= self.min_samples):
                self.sequence += 1
                packet = {'sequence': self.sequence, 'timestamp': (now+self.started)/2,
                          'scan_start': self.started, 'scan_duration': now-self.started,
                          'timing_valid': self.min_duration <= now-self.started <= self.max_duration,
                          'timing_basis': 'host_receipt_midpoint',
                          'points': list(self.points.values()), 'samples': self.samples}
            self.synchronized = True
            self.started = now
            self.points = {}
            self.samples = 0
        elif self.started is not None and now-self.started > self.max_duration:
            # Resynchronize; do not join points separated by a stall.
            self.synchronized = False
            self.points = {}
            self.samples = 0
            self.started = None
        self.previous = a
        self.samples += 1
        key = int(a*2) % 720
        try:
            d = float(point['d_mm'])/1000
            q = float(point.get('q') or 0)
            if math.isfinite(d) and math.isfinite(q) and d>0:
                self.points[key] = (q, a, d)
            else:
                self.points.pop(key, None)
        except (TypeError, ValueError, KeyError):
            self.points.pop(key, None)
        return packet


async def acquisition(cfg, out, stop):
    from rplidarc1 import RPLidar
    lidar = None
    task = None
    try:
        lidar = RPLidar(cfg['port'], cfg['baud'], timeout=0.2)
        # Unbounded Queue.put() does not reliably yield in the supplied driver.
        lidar.output_queue = asyncio.Queue(maxsize=128)
        task = asyncio.create_task(lidar.simple_scan())
        builder = Revolution(cfg['max_scan_duration_s'], cfg['min_revolution_samples'], cfg.get('min_scan_duration_s', .05))
        while not stop.is_set():
            if task.done():
                if task.cancelled():
                    raise RuntimeError('C1 scan task cancelled')
                exc = task.exception()
                if exc:
                    raise exc
                raise RuntimeError('C1 scan task ended')
            try:
                p = await asyncio.wait_for(lidar.output_queue.get(), timeout=0.1)
            except asyncio.TimeoutError:
                continue
            packet = builder.feed(p, time.monotonic())
            if packet is not None:
                latest_put(out, packet)
    finally:
        if lidar:
            lidar.stop_event.set()
        if task:
            try:
                await asyncio.wait_for(task, timeout=1.0)
            except (Exception, asyncio.CancelledError):
                task.cancel()
                try:
                    await task
                except (Exception, asyncio.CancelledError):
                    pass
        if lidar:
            try:
                lidar.shutdown()
            except Exception:
                pass


def child_entry(cfg, out, stop):
    logging.getLogger('RPLidar').setLevel(logging.ERROR)
    try:
        asyncio.run(acquisition(cfg, out, stop))
    except Exception as exc:
        latest_put(out, {'error': f'{type(exc).__name__}: {exc}'})


class LidarReader:
    def __init__(self, cfg, target=child_entry):
        self.cfg, self.target = cfg, target
        self.ctx = mp.get_context('spawn')
        self.process = None
        self.out = None
        self.stop_event = None
        self.latest = None
        self.started = 0
        self.last_receive = 0
        self.generation = 0
        self.status = 'Not started'
        self.next_start = 0

    def start(self):
        self.generation += 1
        self.out = self.ctx.Queue(maxsize=2)
        self.stop_event = self.ctx.Event()
        self.process = self.ctx.Process(target=self.target, args=(self.cfg, self.out, self.stop_event), daemon=True)
        self.process.start()
        self.started = time.monotonic()
        self.last_receive = 0
        self.latest = None
        self.status = 'Starting C1'

    def poll(self, now=None):
        now = time.monotonic() if now is None else now
        if self.process is None:
            if now>=self.next_start:
                self.start()
            return None
        error = None
        try:
            while True:
                value = self.out.get_nowait()
                if 'error' in value:
                    error = value['error']
                else:
                    value['generation'] = self.generation
                    self.latest = value
                    self.last_receive = now
                    self.status = 'C1 receiving'
        except queue.Empty:
            pass
        limit = self.cfg['restart_after_s'] if self.last_receive else self.cfg['startup_timeout_s']
        base = self.last_receive or self.started
        if error or not self.process.is_alive() or now-base>limit:
            reason = error or ('C1 worker exited' if not self.process.is_alive() else 'C1 stream timed out')
            self.status = reason+'; retrying'
            print('[LIDAR]', self.status, flush=True)
            self.close()
            self.latest = None
            self.next_start = time.monotonic()+1.0
            return None
        return self.latest

    def close(self):
        if self.process is not None:
            self.stop_event.set()
            self.process.join(timeout=1.3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1.0)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=1.0)
            if self.process.is_alive():
                raise RuntimeError('Could not stop C1 acquisition process')
            self.process.close()
            self.process = None
        if self.out is not None:
            self.out.cancel_join_thread()
            self.out.close()
            self.out = None
