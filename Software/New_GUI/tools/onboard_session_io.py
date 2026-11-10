"""Read-only onboard MCAP/session/tlog IO. Optional decoders are imported lazily."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re

import numpy as np

from flight_log_io import dependency, resolve_input
from onboard_replay import stream_stats


def header_ns(message):
    stamp=message.header.stamp
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def _median_int(values):
    ordered=sorted(map(int,values))
    if not ordered:return 0
    k=len(ordered)//2
    return ordered[k] if len(ordered)%2 else (ordered[k-1]+ordered[k])//2


@dataclass
class SessionData:
    root: Path
    metadata: dict
    ros_params: dict
    rig_text: str
    gui_config: dict
    imu_ns: np.ndarray
    acc: np.ndarray
    gyro: np.ndarray
    imu_log_ns: np.ndarray
    device_log_ns: np.ndarray
    device_us: np.ndarray
    att_ns: np.ndarray
    att_q: np.ndarray
    att_log_ns: np.ndarray
    attitude_source: str
    ranges: dict
    range_log_ns: dict
    odom_ns: np.ndarray
    odom_p: np.ndarray
    odom_q: np.ndarray
    odom_v_body: np.ndarray
    odom_std: np.ndarray
    odom_log_ns: np.ndarray
    statuses: dict
    streams: dict
    clock: dict
    audit: dict

    def boot_seconds(self, stamps_ns, log_ns=None):
        """Subtract integer clock offsets before converting to seconds."""
        values=np.asarray(stamps_ns,dtype=np.int64)
        segments=self.clock.get('segments',[])
        if len(segments)<=1 or log_ns is None:
            return (values-int(self.clock['offset_ns']))*1e-9
        starts=np.array([s['start_log_ns'] for s in segments],dtype=np.int64)
        offsets=np.array([s['offset_ns'] for s in segments],dtype=np.int64)
        which=np.clip(np.searchsorted(starts,np.asarray(log_ns,dtype=np.int64),side='right')-1,0,len(starts)-1)
        return (values-offsets[which])*1e-9


def clock_mapping(imu_ns,imu_log_ns,device_us,device_log_ns):
    """One-to-one nearest receive-time pairing; report raw offset spread/resets."""
    imu_ns=np.asarray(imu_ns,dtype=np.int64)
    imu_log_ns=np.asarray(imu_log_ns,dtype=np.int64)
    device_log_ns=np.asarray(device_log_ns,dtype=np.int64)
    if not len(imu_ns) or not len(device_us):
        return dict(available=False,offset_ns=0,spread_ns=None,resets=0,pairs=0,segments=[],
                    warning='No paired IMU/device-time stream; PX4 boot alignment unavailable')
    offsets,logs=[],[]
    used=set()
    for i,log in enumerate(imu_log_ns):
        j=int(np.searchsorted(device_log_ns,log))
        candidates=[k for k in (j-1,j,j+1) if 0<=k<len(device_us) and k not in used]
        if not candidates:continue
        k=min(candidates,key=lambda k:abs(int(device_log_ns[k])-int(log)))
        if abs(int(device_log_ns[k])-int(log))>50_000_000:continue
        us=int(device_us[k])
        if us<0 or us*1000>np.iinfo(np.int64).max:continue
        used.add(k);offsets.append(int(imu_ns[i])-us*1000);logs.append(int(log))
    if not offsets:
        return dict(available=False,offset_ns=0,spread_ns=None,resets=0,pairs=0,segments=[],warning='No receive-time matches within 50 ms')
    cuts=[0]+[i for i in range(1,len(offsets)) if abs(offsets[i]-offsets[i-1])>1_000_000]+[len(offsets)]
    segments=[]
    for a,b in zip(cuts,cuts[1:]):
        segments.append(dict(start_log_ns=logs[a],end_log_ns=logs[b-1],offset_ns=_median_int(offsets[a:b]),
                             spread_ns=max(offsets[a:b])-min(offsets[a:b]),pairs=b-a))
    return dict(available=True,offset_ns=_median_int(offsets),spread_ns=max(offsets)-min(offsets),
                resets=len(segments)-1,pairs=len(offsets),unpaired_imu=len(imu_ns)-len(offsets),segments=segments,
                method='Nearest one-to-one MCAP log-time pairing within 50 ms; integer median; jumps >1 ms')


def _vector(v):return (float(v.x),float(v.y),float(v.z))
def _quaternion(q):return (float(q.w),float(q.x),float(q.y),float(q.z))


def read_session(path):
    root=resolve_input(path)
    if root.is_file():root=root.parent
    if root.name=='bag':root=root.parent
    def read_json(name,default=None):
        p=root/name
        return json.loads(p.read_text(encoding='utf-8-sig')) if p.is_file() else ({} if default is None else default)
    params=read_json('config/eskf.yaml')['uwb_eskf']['ros__parameters']
    rig=(root/'config/rig_onboard.txt').read_text(encoding='utf-8-sig')
    metadata,gui=read_json('session.json'),read_json('config/gui_config.json')
    files=list((root/'bag').glob('*.mcap'))
    if not files:raise ValueError('No MCAP files under session/bag')
    def file_order(p):
        digits=re.search(r'(\d+)(?=\.mcap$)',p.name)
        return (int(digits[1]) if digits else -1,p.name)
    files.sort(key=file_order)
    reader=dependency('mcap.reader','mcap')
    factory=dependency('mcap_ros2.decoder','mcap-ros2-support').DecoderFactory()
    exceptions=dependency('mcap.exceptions','mcap')
    rows,topic_times,topic_clocks={}, {}, {}
    warnings=[];file_counts={};zero_topics=set();schemas={}
    selected={'/px4/imu_frd','/px4/imu_device_time_us','/px4/attitude_frd','/px4/odometry',
              '/uwb_eskf/odom','/uwb_eskf/status','/px4/status','/uwb_relay/status','/px4/snapshot','/uwb_eskf/session'}
    for index,path in enumerate(files):
        count=0
        try:
            with path.open('rb') as f:
                try:
                    summary=reader.make_reader(f).get_summary()
                    if summary:zero_topics.update(ch.topic for ch in summary.channels.values())
                except (exceptions.McapError,EOFError):
                    pass
                f.seek(0)
                stream=reader.NonSeekingReader(f)
                for schema,channel,message in stream.iter_messages(log_time_order=False):
                    topic=channel.topic;log=int(message.log_time);count+=1
                    topic_times.setdefault(topic,[]).append((log,log))
                    topic_clocks.setdefault(topic,'log_time')
                    if topic not in selected and not re.fullmatch(r'/uwb/anchor_\d+/range',topic):continue
                    decoder=factory.decoder_for(channel.message_encoding,schema)
                    if decoder is None:raise ValueError(f'No ROS decoder for {topic}')
                    decoded=decoder(message.data)
                    stamp=header_ns(decoded) if hasattr(decoded,'header') else log
                    topic_times[topic][-1]=(log,stamp)
                    topic_clocks[topic]='header' if hasattr(decoded,'header') else 'log_time'
                    rows.setdefault(topic,[]).append((log,stamp,decoded))
                    schemas[topic]=schema.name
        except (exceptions.McapError,EOFError) as exc:
            if index!=len(files)-1:raise
            warnings.append(dict(code='INCOMPLETE_FINAL_MCAP',file=str(path),error=type(exc).__name__,messages_recovered=count))
        file_counts[path.name]=count
    for topic in rows:rows[topic].sort(key=lambda row:row[0])
    streams={}
    for topic in sorted(set(topic_times)|zero_topics):
        ordered=sorted(topic_times.get(topic,[]),key=lambda row:row[0])
        stamps=np.array([r[1] for r in ordered],dtype=np.int64)
        streams[topic]={**stream_stats(stamps),'clock':topic_clocks.get(topic,'log_time'),'schema':schemas.get(topic),
                        'first_log_ns':ordered[0][0] if ordered else None,'last_log_ns':ordered[-1][0] if ordered else None}
    def extract(topic,fn,width=None,dtype=float):
        items=rows.get(topic,[])
        logs=np.array([r[0] for r in items],dtype=np.int64);stamps=np.array([r[1] for r in items],dtype=np.int64)
        values=np.asarray([fn(r[2]) for r in items],dtype=dtype)
        if width is not None:values=values.reshape((-1,width))
        return stamps,values,logs
    imu_ns,acc,imu_log=extract('/px4/imu_frd',lambda m:_vector(m.linear_acceleration),3)
    _,gyro,_=extract('/px4/imu_frd',lambda m:_vector(m.angular_velocity),3)
    _,device_us,device_log=extract('/px4/imu_device_time_us',lambda m:int(m.data),dtype=np.uint64)
    use_att=bool(rows.get('/px4/attitude_frd'))
    if use_att:
        att_ns,att_q,att_log=extract('/px4/attitude_frd',lambda m:_quaternion(m.quaternion),4)
        bad=0
        for i,row in enumerate(rows['/px4/attitude_frd']):
            if row[2].header.frame_id!='px4_local_ned':att_q[i]=np.nan;bad+=1
        attitude_source='attitude_frd, device-stamped'
        if bad:warnings.append(dict(code='ATTITUDE_FRAME_REJECTED',samples=bad))
    else:
        att_ns,att_q,att_log=extract('/px4/odometry',lambda m:_quaternion(m.pose.pose.orientation),4)
        attitude_source='odometry, receive-stamped'
    odom_ns,odom_p,odom_log=extract('/uwb_eskf/odom',lambda m:_vector(m.pose.pose.position),3)
    _,odom_q,_=extract('/uwb_eskf/odom',lambda m:_quaternion(m.pose.pose.orientation),4)
    _,odom_v,_=extract('/uwb_eskf/odom',lambda m:_vector(m.twist.twist.linear),3)
    _,odom_std,_=extract('/uwb_eskf/odom',lambda m:np.sqrt(np.maximum(np.asarray(m.pose.covariance)[[0,7,14]],0)),3)
    ranges,range_logs={},{}
    for topic in rows:
        match=re.fullmatch(r'/uwb/anchor_(\d+)/range',topic)
        if match:
            ns,value,logs=extract(topic,lambda m:float(m.range));ranges[int(match[1])]=(ns,value);range_logs[int(match[1])]=logs
    statuses={}
    for topic in ('/uwb_eskf/status','/px4/status','/uwb_relay/status','/px4/snapshot','/uwb_eskf/session'):
        entries=[]
        for log,_,message in rows.get(topic,[]):
            try:value=json.loads(message.data)
            except (ValueError,TypeError):value={'unparsed':str(message.data)}
            entries.append((log,value))
        statuses[topic]=entries
    clock=clock_mapping(imu_ns,imu_log,device_us,device_log)
    audit=dict(files=[str(p) for p in files],file_message_counts=file_counts,warnings=warnings,
               session_closed=metadata.get('closed'),read_mode='Streaming physical records; topics ordered by MCAP log_time after read',
               limitations=['Live wall-clock freshness, input arrival timeouts and publication timers are not modeled by replay',
                            'Odometry attitude fallback uses receive stamps rather than PX4 device stamps'] if not use_att else
                           ['Live wall-clock freshness, input arrival timeouts and publication timers are not modeled by replay'])
    return SessionData(root,metadata,params,rig,gui,imu_ns,acc,gyro,imu_log,device_log,device_us,
                       att_ns,att_q,att_log,attitude_source,ranges,range_logs,odom_ns,odom_p,odom_q,odom_v,odom_std,odom_log,
                       statuses,streams,clock,audit)


def read_tlog(path,flight_unix_ns=None,boot=None):
    """Split per-stream PX4 clock resets; select the boot by wall-time overlap.

    Sub-0.5 s backwards packets are counted as reorder, not a new power cycle.
    Multiple message types can have different sample latency; they never share
    one last-device-time variable. Fields absent from the dialect remain absent.
    """
    path=resolve_input(path)
    if not path.is_file():raise ValueError('Expected a local tlog file')
    import os
    os.environ.setdefault('MAVLINK20','1')
    mavutil=dependency('pymavlink.mavutil','pymavlink')
    connection=mavutil.mavlink_connection(str(path),dialect='common',robust_parsing=True)
    wanted={'HIGHRES_IMU','ATTITUDE_QUATERNION','OPTICAL_FLOW_RAD','DISTANCE_SENSOR','VIBRATION'}
    boots=[];current=None;clocks={};reordered=0
    try:
        while True:
            message=connection.recv_match()
            if message is None:break
            kind=message.get_type()
            if kind not in wanted:continue
            fields=message.to_dict()
            if kind=='DISTANCE_SENSOR':
                fields.setdefault('signal_quality',None)  # MAVLink extension absent in older dialects.
            device=(fields.get('time_boot_ms')/1000 if fields.get('time_boot_ms') is not None else
                    fields.get('time_usec')/1e6 if fields.get('time_usec') is not None else None)
            wall=getattr(message,'_timestamp',None)
            if wall is None or device is None or not math.isfinite(wall) or not math.isfinite(device):continue
            previous=clocks.get(kind)
            reset=previous is not None and device<previous-.5
            if current is None or reset:
                current=dict(index=len(boots)+1,start_unix_s=float(wall),end_unix_s=float(wall),
                             boot_min_s=float(device),boot_max_s=float(device),messages={})
                boots.append(current);clocks={}
            elif previous is not None and device<previous:reordered+=1
            clocks[kind]=max(clocks.get(kind,device),device)
            current['end_unix_s']=float(wall)
            current['boot_min_s']=min(current['boot_min_s'],float(device));current['boot_max_s']=max(current['boot_max_s'],float(device))
            current['messages'].setdefault(kind,[]).append(dict(unix_s=float(wall),boot_s=float(device),fields=fields))
    finally:connection.close()
    if not boots:raise ValueError('No timed IMU/attitude/flow/range messages in tlog')
    if boot is not None:
        if not 1<=boot<=len(boots):raise ValueError('Boot selection is 1-based and out of range')
        selected=boots[boot-1]
    elif flight_unix_ns is not None:
        lo,hi=(int(v)/1e9 for v in flight_unix_ns)
        overlap=[max(0.,min(hi,b['end_unix_s'])-max(lo,b['start_unix_s'])) for b in boots]
        if max(overlap)<=0:raise ValueError('No tlog boot overlaps the session wall-time span; specify --boot')
        selected=boots[int(np.argmax(overlap))]
    else:
        if len(boots)>1:raise ValueError('Multiple boots require a session wall-time span or --boot')
        selected=boots[0]
    return dict(path=str(path),boots=boots,selected_boot=selected['index'],selected=selected,
                reordered_packets=reordered,selection='Explicit --boot' if boot is not None else 'Maximum overlap with session MCAP log-time span',
                limitations=['Boot detection uses per-message-type decreases >0.5 s; smaller backwards packets counted as reorder',
                            'Apparent rates depend on the recorded MAVLink streams; flow is not fused'])
