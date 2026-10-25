"""Publish actual TAG telemetry received over UDP, with no sampling timer.

Burst results and slow RANGE snapshots have distinct topics. This diagnostic
adapter keeps raw/unvalidated results out of the calibrated /range topics.
Range.header.stamp is host receive time; TAG time is preserved in cycle topics
and /uwb/burst_metadata. This tool cannot establish the tester node's behavior.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from collections import Counter, deque
import json
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import telemetry_protocol as tp
from burst_metadata import BurstMetadata
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String, UInt32, UInt64


class Bridge(Node):
    def __init__(self):
        super().__init__('uwb_uart_burst_bridge')
        self.qos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE)
        self.pubs={}
        self.seen=set()
        self.order=deque()
        self.counts=Counter()
        self.first=None
        self.builder=BurstMetadata()
        self.last=None
        self.metadata=self.create_publisher(String,'/uwb/burst_metadata',self.qos)
        self.cycle=self.create_publisher(UInt32,'/uwb/cycle_seq',self.qos)
        self.tag_time=self.create_publisher(UInt64,'/uwb/tag_time_us',self.qos)
        self.period=self.create_publisher(UInt32,'/uwb/period_us',self.qos)
        self.device=self.create_publisher(String,'/uwb/device_info',self.qos)

    def range_message(self,topic,anchor,value_mm,stamp):
        if topic not in self.pubs:
            self.pubs[topic]=self.create_publisher(Range,topic,self.qos)
        msg=Range()
        msg.header.stamp=stamp
        msg.header.frame_id=f'uwb_anchor_{anchor}'
        # Range has no UWB radiation enum; consumers use only header/range here.
        msg.field_of_view=0.0
        msg.min_range=0.0
        msg.max_range=65.534
        msg.range=value_mm/1000.0
        self.pubs[topic].publish(msg)

    def accept(self,msg):
        self.builder.accept(msg)
        stamp=self.get_clock().now().to_msg()
        if isinstance(msg,tp.DeviceInfoMessage):
            self.device.publish(String(data=json.dumps(dict(boot_id=msg.boot_id,
                config_hash=hex(msg.build_config_hash),features=msg.telemetry_features,
                calibrated_mask=msg.calibrated_mask,uart_baud=msg.uart_baud))))
        elif isinstance(msg,tp.RangeMessage):
            self.counts['snapshot']+=1
            for a in msg.samples:
                if not a.status & (tp.STATUS_TIMEOUT|tp.STATUS_RX_ERROR|tp.STATUS_COMPUTE_ERROR):
                    self.range_message(f'/uwb/snapshot/anchor_{a.anchor_id}/diagnostic_range',
                                       a.anchor_id,a.raw_mm,stamp)
        elif isinstance(msg,tp.RangeBurstMessage):
            key=(msg.boot_id,msg.cycle_seq)
            if key in self.seen:
                self.counts['duplicate_burst']+=1
                return
            self.seen.add(key)
            self.order.append(key)
            if len(self.order)>4096:
                self.seen.remove(self.order.popleft())
            if self.last is not None and self.last.boot_id != msg.boot_id:
                self.first=None
            if self.first is None:
                self.first=msg
            self.last=msg
            self.counts['burst']+=1
            self.cycle.publish(UInt32(data=msg.cycle_seq))
            self.tag_time.publish(UInt64(data=msg.t_us))
            self.period.publish(UInt32(data=msg.period_us))
            ds_count=0
            for a in msg.anchors:
                cal=bool(a.flags&tp.BURST_FLAG_CAL_OK)
                valid=bool(a.flags&tp.BURST_FLAG_VALID)
                if not a.radio_ok or a.mode_name!='DS' or a.range_mm is None:
                    continue
                ds_count+=1
                self.counts[f'ds_a{a.anchor_id}']+=1
                self.range_message(f'/uwb/anchor_{a.anchor_id}/diagnostic_range',
                                   a.anchor_id,a.range_mm,stamp)
                if cal and valid:
                    self.range_message(f'/uwb/anchor_{a.anchor_id}/range',
                                       a.anchor_id,a.range_mm,stamp)
            if ds_count==8:
                self.counts['all8_ds']+=1
            self.metadata.publish(String(data=json.dumps(self.builder.build(msg),
                separators=(',',':'),allow_nan=False)))

    def report(self):
        device_hz=None
        if self.first is not None and self.last.t_us>self.first.t_us:
            device_hz=(self.last.cycle_seq-self.first.cycle_seq)*1e6/(self.last.t_us-self.first.t_us)
        self.get_logger().info(f'counts={dict(self.counts)}, TAG cycle Hz={device_hz}')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bind',default='0.0.0.0')
    ap.add_argument('--udp-port',type=int,default=15555)
    ap.add_argument('--port',help='Read UART directly instead of UDP, e.g. /dev/ttyACM0')
    ap.add_argument('--baud',type=int,default=460800)
    args,ros_args=ap.parse_known_args()
    rclpy.init(args=ros_args)
    node=Bridge()
    parser=tp.TelemetryStreamParser()
    last_report=0.0
    try:
        with ExitStack() as stack:
            if args.port:
                import serial
                uart=stack.enter_context(serial.Serial(args.port,115200,timeout=0.05))
                uart.baudrate=args.baud  # force J-Link VCOM line-coding transition
                seq=int(time.time_ns())&0xFFFFFFFF
                uart.write(tp.encode_command(seq,tp.CMD_SET_TELEMETRY,bytes((7,))))
                uart.write(tp.encode_command((seq+1)&0xFFFFFFFF,tp.CMD_GET_DEVICE_INFO))
                node.get_logger().info(f'Listening UART {args.port} at {args.baud}; telemetry 0x07 requested')
            else:
                udp=stack.enter_context(socket.socket(socket.AF_INET,socket.SOCK_DGRAM))
                udp.bind((args.bind,args.udp_port))
                udp.settimeout(0.1)
                node.get_logger().info(f'Listening UDP {args.bind}:{args.udp_port}; no synthetic samples')
            while rclpy.ok():
                try:
                    if args.port:
                        packet=uart.read(min(uart.in_waiting or 1,65535))
                    else:
                        packet,_=udp.recvfrom(65535)
                except socket.timeout:
                    packet=b''
                for frame in parser.feed(packet):
                    try:
                        node.accept(tp.decode_frame(frame))
                    except tp.ProtocolError as error:
                        node.get_logger().warning(str(error))
                rclpy.spin_once(node,timeout_sec=0)
                now=time.monotonic()
                if now-last_report>=1:
                    node.report()
                    last_report=now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
