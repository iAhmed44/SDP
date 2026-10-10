#!/usr/bin/env python3
"""Subscribe to the sensing NavBridge. No sensor, GPIO or model access."""
import argparse
import json
import time
from comms.nav_bridge import SnapshotReceiver


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connect',default='tcp://127.0.0.1:5555')
    p.add_argument('--duration',type=float,default=0,help='Seconds; 0 runs until Ctrl+C')
    args=p.parse_args()
    import zmq
    ctx=zmq.Context();sock=ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER,0)
    sock.setsockopt(zmq.RCVHWM,2)
    sock.setsockopt_string(zmq.SUBSCRIBE,'NAV_STATIC ')
    sock.connect(args.connect)
    receiver=SnapshotReceiver();start=time.monotonic();previous=None
    print('[NAV TEST] Waiting for publisher. INVALID means unknown, not clear space.')
    try:
        while args.duration<=0 or time.monotonic()-start<args.duration:
            if sock.poll(100):
                # Drain pending messages, retaining only the newest accepted state.
                for _ in range(100):
                    try:message=sock.recv_string(zmq.NOBLOCK)
                    except zmq.Again:break
                    try:receiver.accept(message)
                    except (ValueError,KeyError,TypeError) as exc:print('[REJECTED]',exc)
            current=receiver.current()
            signature=(current.get('session_id'),current.get('sequence'),current['valid'])
            if signature!=previous:
                print(json.dumps(current,indent=2),flush=True);previous=signature
    except KeyboardInterrupt:pass
    finally:sock.close();ctx.term()


if __name__=='__main__':main()
