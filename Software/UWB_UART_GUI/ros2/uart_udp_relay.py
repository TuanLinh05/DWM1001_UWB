"""Windows UART -> WSL UDP, one CRC-checked TAG frame per datagram.

Only telemetry/identity/heartbeat commands are sent. Burst timing is unchanged.
Raw frame capture and command acknowledgements are saved for a later audit.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import telemetry_protocol as tp
import serial


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True, help='Windows COM port, e.g. COM14')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--host', required=True, help='WSL IPv4 from wsl hostname -I')
    ap.add_argument('--udp-port', type=int, default=15555)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    if args.baud < tp.TELEM_RANGE_MEAS_MIN_BAUD:
        ap.error('Measurement telemetry requires baud >= 460800')
    stamp = datetime.now(timezone(timedelta(hours=7))).strftime('%Y%m%d_%H%M%S')
    out = args.output or Path(__file__).resolve().parents[1]/'Output/ros2_logs'/f'{stamp}_{args.port}'
    out.mkdir(parents=True, exist_ok=False)
    parser = tp.TelemetryStreamParser()
    counts = Counter()
    command_seq = 0
    configured = False
    identity = None
    started = time.monotonic()
    last_status = last_ping = last_identity = 0.0
    session = dict(source=args.port, baud=args.baud, udp_host=args.host,
                   udp_port=args.udp_port, started_at=datetime.now(timezone.utc).isoformat())

    def write_session():
        session.update(duration_s=time.monotonic()-started,
                       frames={hex(k):v for k,v in counts.items()},
                       parser=asdict(parser.counters), device_info=identity,
                       measurement_telemetry_ack=configured)
        (out/'session.json').write_text(json.dumps(session,indent=2),encoding='utf-8')

    print(f'Capture: {out}\nUDP -> {args.host}:{args.udp_port}', flush=True)
    try:
        with serial.Serial(args.port,args.baud,timeout=0.05) as uart, \
             socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp, \
             (out/'raw_telemetry.bin').open('wb') as raw, \
             (out/'commands.jsonl').open('w',encoding='utf-8') as commands:
            def send(command, payload=b''):
                nonlocal command_seq
                command_seq += 1
                uart.write(tp.encode_command(command_seq,command,payload))

            uart.reset_input_buffer()
            while True:
                now = time.monotonic()
                if now-last_ping >= 1.0:
                    send(tp.CMD_PING)
                    last_ping = now
                if identity is None and now-last_identity >= 2.0:
                    send(tp.CMD_GET_DEVICE_INFO)
                    last_identity = now
                for frame in parser.feed(uart.read(uart.in_waiting or 1)):
                    encoded = tp.encode_frame(frame.type,frame.sequence,frame.time_ms,frame.payload)
                    raw.write(encoded)
                    counts[frame.type] += 1
                    udp.sendto(encoded,(args.host,args.udp_port))
                    try:
                        msg=tp.decode_frame(frame)
                    except tp.ProtocolError:
                        parser.counters.decode_errors += 1
                        continue
                    if isinstance(msg,tp.DeviceInfoMessage):
                        first = identity is None or identity['boot_id'] != msg.boot_id
                        identity=asdict(msg)
                        if first:
                            configured = False
                            print(f'TAG config=0x{msg.build_config_hash:08X}, '
                                  f'features=0x{msg.telemetry_features:X}',flush=True)
                            send(tp.CMD_SET_TELEMETRY,bytes((0x07,)))
                    elif isinstance(msg,tp.CmdAckMessage):
                        event=asdict(msg)
                        event['data']=msg.data.hex()
                        commands.write(json.dumps(event)+'\n')
                        if msg.command_id == tp.CMD_SET_TELEMETRY:
                            configured = msg.ok and bool(msg.data) and bool(msg.data[0]&0x02)
                            print(f'SET_TELEMETRY result={msg.result}, data={msg.data.hex()}',flush=True)
                if now-last_status >= 1.0:
                    print(f'Frames: burst={counts[0x18]}, snapshot={counts[0x01]}, '
                          f'CRC errors={parser.counters.crc_errors}',flush=True)
                    raw.flush()
                    commands.flush()
                    write_session()
                    last_status=now
    except KeyboardInterrupt:
        print('Stopped. Capture saved.',flush=True)
    finally:
        session['ended_at']=datetime.now(timezone.utc).isoformat()
        write_session()


if __name__=='__main__':
    main()
