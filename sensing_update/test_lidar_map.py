#!/usr/bin/env python3
"""LiDAR diagnostic viewer only. This entry point never enables GPIO."""
import sys
from main_sensing import main

if __name__ == '__main__':
    if '--motors' in sys.argv:
        raise SystemExit('The LiDAR viewer does not enable motors. Use main_sensing.py --motors for integration.')
    sys.argv.append('--lidar-only')
    main()
