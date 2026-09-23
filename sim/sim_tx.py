#!/usr/bin/env python3
"""SITL transmitter: the RC link PX4 sees, switches flipped by the pilot.

Sends RC_CHANNELS_OVERRIDE to PX4 SITL's GCS MAVLink port at 20 Hz, which
PX4 takes as RC input exactly like a receiver (launch sitl with rc:=tx for
the channel mapping):

    ch1-4  sticks: roll/pitch/yaw centred, throttle LOW
    ch5    ARM switch         (1000 off, 2000 on)
    ch6    OFFBOARD switch    (1000 off, 2000 on)

It does nothing on its own. The switches are the pilot's:

    python3 sim_tx.py run                 # the transmitter (leave running)
    python3 sim_tx.py set offboard on     # flip a switch
    python3 sim_tx.py set arm on
    python3 sim_tx.py set arm off

Switch positions live in ~/.sim_tx (one line: "arm=0 offboard=0").
"""

import os
import sys
import time

STATE = os.path.expanduser('~/.sim_tx')


def read_state():
    st = {'arm': 0, 'offboard': 0}
    try:
        for tok in open(STATE).read().split():
            k, v = tok.split('=')
            st[k] = int(v)
    except (OSError, ValueError):
        pass
    return st


def write_state(st):
    tmp = STATE + '.tmp'
    with open(tmp, 'w') as f:
        f.write(' '.join(f'{k}={v}' for k, v in st.items()) + '\n')
    os.replace(tmp, STATE)


def run(url='udpout:127.0.0.1:18570'):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
    write_state(read_state())
    last_hb = 0.0
    last = None
    print(f"sim_tx: transmitter on {url}. Switches: {STATE}", flush=True)
    while True:
        now = time.monotonic()
        if now - last_hb >= 1.0:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = now
        st = read_state()
        if st != last:
            print(f"sim_tx: arm={'ON' if st['arm'] else 'off'} "
                  f"offboard={'ON' if st['offboard'] else 'off'}", flush=True)
            last = st
        ch5 = 2000 if st['arm'] else 1000
        ch6 = 2000 if st['offboard'] else 1000
        m.mav.rc_channels_override_send(1, 1, 1500, 1500, 1000, 1500,
                                        ch5, ch6, 1500, 1500)
        time.sleep(0.05)


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == 'run':
        run(*sys.argv[2:3])
    elif len(sys.argv) == 4 and sys.argv[1] == 'set' and sys.argv[2] in ('arm', 'offboard'):
        st = read_state()
        st[sys.argv[2]] = 1 if sys.argv[3] in ('on', '1') else 0
        write_state(st)
        print(f"{sys.argv[2]} -> {'ON' if st[sys.argv[2]] else 'off'}")
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == '__main__':
    main()
