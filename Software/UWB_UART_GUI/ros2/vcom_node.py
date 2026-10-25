"""Direct-serial calibration-console adapter; run with --port /dev/ttyACM0.

Keep alongside uwb_ros_bridge.py and the parent telemetry_protocol.py /
burst_metadata.py. Also supports the Windows UART -> UDP -> WSL route.
"""
from uwb_ros_bridge import main

if __name__ == '__main__':
    main()
