"""Offline comparison of CRC-checked UART frames and a recorded ROS2 bag.

Run under Ubuntu with /opt/ros/humble/setup.bash sourced. No device access.
UART is a frozen copy when the relay is still writing. ROS2's storage reader
handles MCAP framing and compression; TAG data is decoded independently.
"""
from __future__ import annotations

import argparse
from array import array
import binascii
from collections import Counter, defaultdict
from dataclasses import asdict
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import statistics
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import telemetry_protocol as tp


def describe(values):
    v = sorted(values)
    if not v:
        return None
    def q(p):
        i = (len(v)-1)*p
        lo = int(i)
        return v[lo] + (v[min(lo+1, len(v)-1)]-v[lo])*(i-lo)
    return dict(n=len(v), mean=statistics.fmean(v), median=q(.5),
                p01=q(.01), p99=q(.99), minimum=v[0], maximum=v[-1])


def timing(times, scale=1e9):
    if len(times) < 2:
        return dict(count=len(times))
    dt = [(b-a)/scale for a, b in zip(times, times[1:])]
    span = (times[-1]-times[0])/scale
    return dict(count=len(times), span_s=span,
                rate_hz=(len(times)-1)/span if span > 0 else None,
                interval_s=describe(dt), nonpositive_intervals=sum(t<=0 for t in dt),
                intervals_below_1ms=sum(0<=t<.001 for t in dt),
                intervals_above_50ms=sum(t>.05 for t in dt))


def ds(entry):
    return entry.radio_ok and entry.mode_name == 'DS' and entry.range_mm is not None


def parse_uart(path):
    data = path.read_bytes()
    counts, by_key = Counter(), {}
    cycle, t_us, rtc, periods = array('I'), array('Q'), array('Q'), array('H')
    snapshots = array('Q')
    anchors = {a: dict(times=array('Q'), ranges=array('H'), fp=array('h'),
                      status=Counter(), mode=Counter(), flags=Counter(),
                      resp=0, late=0, calibrated=0, valid=0) for a in range(1,9)}
    diag, first_diag, last_diag = Counter(), {}, {}
    info = defaultdict(list)
    identities, systems, boots, measurement_keys = [], [], Counter(), set()
    duplicate_cycles = duplicate_meas = all8 = 0
    pos = 0
    while pos+tp.HEADER_SIZE <= len(data):
        if data[pos:pos+2] != tp.SOF:
            raise ValueError(f'Unexpected framing at byte {pos}')
        version, typ, length, seq, ms = struct.unpack_from('<BBHII', data, pos+2)
        end = pos+tp.HEADER_SIZE+length+tp.CRC_SIZE
        if end > len(data):
            break
        if version != tp.VERSION or length > tp.MAX_PAYLOAD:
            raise ValueError(f'Unsupported frame at byte {pos}')
        if binascii.crc_hqx(data[pos+2:end-2], 0xffff) != struct.unpack_from('<H', data, end-2)[0]:
            raise ValueError(f'Capture CRC mismatch at byte {pos}')
        payload = data[pos+14:end-2]
        counts[typ] += 1
        msg = tp.decode_frame(tp.Frame(version, typ, length, seq, ms, payload))
        if isinstance(msg, tp.RangeBurstMessage):
            key = (msg.boot_id, msg.cycle_seq)
            duplicate_cycles += int(key in by_key)
            by_key[key] = payload
            boots[msg.boot_id] += 1
            cycle.append(msg.cycle_seq)
            t_us.append(msg.t_us)
            rtc.append(ms)
            periods.append(msg.period_us)
            n_ds = 0
            offset = 0
            for a in msg.anchors:
                s = anchors[a.anchor_id]
                s['status'][hex(a.status)] += 1
                s['mode'][a.mode_name] += 1
                s['flags'][hex(a.flags)] += 1
                s['resp'] += int(a.fp_cdbm != tp.INT16_MIN)
                if a.fp_cdbm != tp.INT16_MIN:
                    s['fp'].append(a.fp_cdbm)
                s['late'] += bool(a.flags & tp.BURST_FLAG_ANCHOR_LATE)
                s['calibrated'] += bool(a.flags & tp.BURST_FLAG_CAL_OK)
                s['valid'] += bool(a.flags & tp.BURST_FLAG_VALID)
                if ds(a):
                    n_ds += 1
                    s['times'].append(msg.t_us)
                    s['ranges'].append(a.range_mm)
                    mkey = (msg.boot_id, (msg.meas_seq_first+offset) & 0xffffffff)
                    duplicate_meas += int(mkey in measurement_keys)
                    measurement_keys.add(mkey)
                if a.radio_ok:
                    offset += 1
            all8 += int(n_ds == 8)
        elif isinstance(msg, tp.RangeMessage):
            snapshots.append(ms)
        elif isinstance(msg, tp.DiagAnchorMessage):
            diag[msg.anchor_id] += 1
            first_diag.setdefault(msg.anchor_id, msg)
            last_diag[msg.anchor_id] = msg
        elif isinstance(msg, tp.DiagSystemMessage):
            systems.append(msg)
        elif isinstance(msg, tp.DeviceInfoMessage):
            identities.append(msg)
        elif isinstance(msg, tp.AnchorInfoMessage):
            info[msg.anchor_id].append(msg)
        pos = end
    if len(boots) != 1:
        raise ValueError(f'Expected one boot for this comparison, got {dict(boots)}')
    span = (t_us[-1]-t_us[0])/1e6
    results = {}
    for a, s in anchors.items():
        d0, d1 = first_diag.get(a), last_diag.get(a)
        dspan = (d1.time_ms-d0.time_ms)/1000 if d0 and d1 else None
        count = len(s['times'])
        results[a] = dict(ds_records=count, ds_hz=count/span,
            ds_cycle_pct=100*count/len(cycle), ds_timing=timing(s['times'],1e6),
            response_pct=100*s['resp']/len(cycle), statuses=dict(s['status']),
            modes=dict(s['mode']), flags=dict(s['flags']), late=s['late'],
            calibrated=s['calibrated'], valid=s['valid'], range_mm=describe(s['ranges']),
            fp_dbm=describe(x/100 for x in s['fp']),
            consecutive_range_change_pct=100*sum(x!=y for x,y in zip(s['ranges'],s['ranges'][1:]))/(count-1),
            diag_ds_delta=d1.ds_ok-d0.ds_ok if d0 else None,
            diag_span_s=dspan, diag_ds_hz=(d1.ds_ok-d0.ds_ok)/dspan if dspan else None,
            anchor_config_hashes=sorted({hex(m.build_config_hash) for m in info[a]}),
            anchor_boot_counts=sorted({m.boot_count for m in info[a]}),
            anchor_info_statuses=dict(Counter(hex(m.anchor_status) for m in info[a])))
    system_summary = None
    if systems:
        d0, d1 = systems[0], systems[-1]
        fields = ['cycle_count','cycle_overruns','spi_errors','radio_recoveries',
            'radio_recovery_failures','config_checks','config_mismatches',
            'uart_tx_overflow','uart_rx_overflow','meas_queue_drops','burst_record_drops',
            'burst_final_late','ds_report_timeout','ds_fallback','ds_final_tx_timeout']
        system_summary = dict(count=len(systems), first=asdict(d0), last=asdict(d1),
            deltas={f:getattr(d1,f)-getattr(d0,f) for f in fields},
            rx_error_deltas={f:d1.rx_errors[f]-d0.rx_errors[f] for f in d0.rx_errors})
    summary = dict(source=str(path), bytes=len(data), sha256=hashlib.sha256(data).hexdigest(),
        crc_checked_frames=sum(counts.values()), incomplete_trailing_bytes=len(data)-pos,
        frame_counts={hex(k):v for k,v in sorted(counts.items())}, boots=dict(boots),
        first_cycle=cycle[0], last_cycle=cycle[-1], first_tag_time_us=t_us[0], last_tag_time_us=t_us[-1],
        cycle_tag_time=timing(t_us,1e6), cycle_rtc_time=timing(rtc,1e3),
        generated_cycle_hz=(cycle[-1]-cycle[0])/span,
        missing_cycles=cycle[-1]-cycle[0]+1-len(cycle)+duplicate_cycles,
        duplicate_cycles=duplicate_cycles, duplicate_measurement_keys=duplicate_meas,
        period_us=describe(periods), snapshots=timing(snapshots,1e3),
        all8_ds_cycles=all8, all8_ds_pct=100*all8/len(cycle), all8_ds_hz=all8/span,
        total_ds_records=sum(s['ds_records'] for s in results.values()), anchors=results,
        first_identity=asdict(identities[0]) if identities else None,
        last_identity=asdict(identities[-1]) if identities else None,
        system=system_summary)
    print(f'UART: {len(cycle)} bursts; {summary["generated_cycle_hz"]:.6f} generated cycles/s',flush=True)
    return summary, by_key


def read_bag(path, uart_keys):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    types = {t.name:t.type for t in reader.get_all_topics_and_types()}
    classes = {t:get_message(v) for t,v in types.items()}
    logs = {t:array('Q') for t in types}
    stamps, ranges = {}, {}
    expected_ranges = {a:array('H') for a in range(1,9)}
    meta_cycles, meta_times, meta_logs = array('I'), array('Q'), array('Q')
    counts, seen, identity = Counter(), set(), []
    matched, mismatches, missing_uart = 0, [], []
    all8 = ds_total = 0
    modes, statuses, boot_ids = Counter(), Counter(), Counter()
    first_matched_cycle = last_matched_cycle = None
    tag_topic, cycle_topic, periods = array('Q'), array('I'), Counter()
    progress = 0
    started = time.monotonic()
    while reader.has_next():
        topic, data, logged = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        logs[topic].append(logged)
        counts[topic] += 1
        if types[topic]=='sensor_msgs/msg/Range':
            stamps.setdefault(topic,array('Q')).append(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
            ranges.setdefault(topic,array('f')).append(msg.range)
        elif topic == '/uwb/burst_metadata':
            m = json.loads(msg.data)
            key = (m['boot_id'],m['cycle_seq'])
            if key in seen:
                progress += 1
            seen.add(key)
            boot_ids[m['boot_id']] += 1
            meta_cycles.append(m['cycle_seq'])
            meta_times.append(m['tag_time_us'])
            meta_logs.append(logged)
            all8 += int(m['ds_count']==8)
            ds_total += m['ds_count']
            for a in m['anchors']:
                modes[a['mode']] += 1
                statuses[hex(a['status'])] += 1
                if a['mode']=='DS' and a['flags'] & tp.BURST_FLAG_RADIO_OK and a['range_mm'] is not None:
                    expected_ranges[a['anchor_id']].append(a['range_mm'])
            payload = uart_keys.get(key)
            if payload is None:
                if len(missing_uart)<20:
                    missing_uart.append(key)
            else:
                b = tp.decode_range_burst(tp.Frame(1,tp.TYPE_RANGE_BURST,len(payload),0,0,payload))
                expected = dict(boot_id=b.boot_id,cycle_seq=b.cycle_seq,
                    meas_seq_first=b.meas_seq_first, tag_time_us=b.t_us,
                    period_us=b.period_us, ds_count=sum(ds(a) for a in b.anchors),
                    anchors=[dict(anchor_id=a.anchor_id,mode=a.mode_name,status=a.status,
                        flags=a.flags,range_mm=a.range_mm,
                        calibrated=bool(a.flags&tp.BURST_FLAG_CAL_OK),
                        valid=bool(a.flags&tp.BURST_FLAG_VALID)) for a in b.anchors])
                if m == expected:
                    matched += 1
                    if first_matched_cycle is None:
                        first_matched_cycle = key[1]
                    last_matched_cycle = key[1]
                elif len(mismatches)<20:
                    mismatches.append(dict(key=key,ros=m,uart=expected))
        elif topic == '/uwb/device_info':
            identity.append(json.loads(msg.data))
        elif topic == '/uwb/cycle_seq':
            cycle_topic.append(msg.data)
        elif topic == '/uwb/tag_time_us':
            tag_topic.append(msg.data)
        elif topic == '/uwb/period_us':
            periods[msg.data] += 1
        if sum(counts.values())%500000==0:
            print(f'ROS: {sum(counts.values()):,} messages read, {time.monotonic()-started:.1f}s',flush=True)
    first = min(v[0] for v in logs.values() if v)
    last = max(v[-1] for v in logs.values() if v)
    topics = {}
    for t, times in logs.items():
        topic_summary = dict(type=types[t], bag_time=timing(times))
        if t in stamps:
            topic_summary['header_time'] = timing(stamps[t])
            topic_summary['range_m'] = describe(ranges[t])
            topic_summary['duplicate_header_stamps'] = len(stamps[t])-len(set(stamps[t]))
        topics[t] = topic_summary
    range_compare = {}
    for a, expected in expected_ranges.items():
        topic = f'/uwb/anchor_{a}/diagnostic_range'
        actual = ranges.get(topic, array('f'))
        # Subscribers discover at slightly different times: only allow a small
        # prefix shift; report mismatches rather than silently interpolating.
        best = None
        for shift in range(-4,5):
            i,j = max(0,shift), max(0,-shift)
            n = min(len(expected)-i,len(actual)-j)
            if n<=0:
                continue
            mismatch = sum(abs(expected[i+k]/1000-actual[j+k])>0.00001 for k in range(n))
            candidate = dict(metadata_ds=len(expected), ros_range_count=len(actual),
                expected_prefix_skipped=i, ros_prefix_skipped=j, compared=n,
                value_mismatches=mismatch, expected_suffix_skipped=len(expected)-i-n,
                ros_suffix_skipped=len(actual)-j-n)
            if best is None or mismatch<best['value_mismatches']:
                best = candidate
        range_compare[a] = best
    windows = []
    # Count in disjoint host 60s bins; final partial bin explicitly labelled.
    for start in range(0,int((last-first)/1e9)+1,60):
        stop = min(start+60,(last-first)/1e9)
        n = sum(start<=(t-first)/1e9<stop for t in meta_logs)
        windows.append(dict(start_s=start,end_s=stop,count=n,rate_hz=n/(stop-start) if stop>start else None))
    mspan=(meta_times[-1]-meta_times[0])/1e6
    host_span=(meta_logs[-1]-meta_logs[0])/1e9
    local = timezone(timedelta(hours=7))
    matched_window_uart = sum(first_matched_cycle<=k[1]<=last_matched_cycle
                              and k[0] in boot_ids for k in uart_keys)
    return dict(source=str(path), message_count=sum(counts.values()),
        start_iso=datetime.fromtimestamp(first/1e9,local).isoformat(),
        end_iso=datetime.fromtimestamp(last/1e9,local).isoformat(), duration_s=(last-first)/1e9,
        topics=topics, metadata=dict(count=len(meta_cycles),boot_ids=dict(boot_ids),
            first_cycle=meta_cycles[0],last_cycle=meta_cycles[-1],
            first_tag_time_us=meta_times[0],last_tag_time_us=meta_times[-1],
            duplicates=progress,missing_cycles=meta_cycles[-1]-meta_cycles[0]+1-len(meta_cycles)+progress,
            tag_time=timing(meta_times,1e6), host_time=timing(meta_logs),
            generated_cycle_hz=(meta_cycles[-1]-meta_cycles[0])/mspan,
            all8_ds_cycles=all8,all8_ds_pct=100*all8/len(meta_cycles),all8_ds_hz=all8/mspan,
            ds_records=ds_total,modes=dict(modes),statuses=dict(statuses),
            matched_uart=matched,uart_missing_first20=missing_uart,mismatches_first20=mismatches,
            uart_cycles_in_same_window=matched_window_uart,
            uart_not_in_ros_window=matched_window_uart-matched),
        range_value_comparison=range_compare, host_60s_bins=windows,
        cycle_topic_count=len(cycle_topic),tag_topic_count=len(tag_topic),
        cycle_topic_same_as_metadata=list(cycle_topic)==list(meta_cycles),
        tag_topic_same_as_metadata=list(tag_topic)==list(meta_times),
        period_us_values=dict(periods), identities=identity,
        host_to_tag_span_ratio=host_span/mspan,
        host_minus_tag_span_s=host_span-mspan)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--uart',type=Path,required=True)
    ap.add_argument('--bag',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    uart, keys=parse_uart(args.uart)
    (args.out/'uart_analysis.json').write_text(json.dumps(uart,indent=2),encoding='utf-8')
    ros=read_bag(args.bag,keys)
    result=dict(uart=uart,ros=ros)
    (args.out/'paired_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(uart_cycles=uart['cycle_tag_time']['count'],
        tag_hz=uart['generated_cycle_hz'],ros_metadata=ros['metadata'],
        range_comparison=ros['range_value_comparison'],
        host_clock_ratio=ros['host_to_tag_span_ratio']),indent=2),flush=True)


if __name__=='__main__':
    main()
