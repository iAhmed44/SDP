#!/usr/bin/env python3
"""Two-pulse check on one motor; no camera, LiDAR or Navigation access.
Stop the full sensing program first. GPIO is disabled unless --motors is given.
"""
import argparse
from pathlib import Path
import signal
import threading
import time
from main_sensing import load_config
from haptics.motor_driver import MotorDriver
from version import VERSION


def main():
    base=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(base/'config.json'))
    parser.add_argument('--index',type=int,choices=range(5),default=2,help='0 FL, 1 L, 2 center, 3 R, 4 FR')
    parser.add_argument('--motors',action='store_true')
    args=parser.parse_args();h=load_config(args.config)['haptics']
    stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    driver=MotorDriver(h,args.motors)
    print(f"Sensing {VERSION}: motor index {args.index}, GPIO {h['pins'][args.index]}, {'ENABLED' if args.motors else 'DRY RUN'}",flush=True)
    try:
        for phase,duration,duty in [('pulse1',h['departure_pulse_s'],h['departure_duty']),
                                    ('gap',h['departure_gap_s'],0),
                                    ('pulse2',h['departure_pulse_s'],h['departure_duty'])]:
            if stop.is_set():break
            print(f'{phase}: {duration*1000:.0f}ms, duty {duty*100:.0f}%',flush=True)
            end=time.monotonic()+duration
            while not stop.is_set() and time.monotonic()<end:
                threats=[{'angle_deg':-90+45*args.index,'dist_m':0,'duty_override':duty}] if duty else []
                driver.apply_feedback(threats,valid_until=end)
                stop.wait(min(.01,max(0,end-time.monotonic())))
    finally:
        success=driver.close()
        print('Zero commands: '+('successful' if success else 'FAILED'),flush=True)


if __name__=='__main__':main()
