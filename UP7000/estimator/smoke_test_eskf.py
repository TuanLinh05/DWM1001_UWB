#!/usr/bin/env python3
"""30 s real-time synthetic ROS smoke test; no serial/MAVLink/drone connection.

Run in an isolated domain (e.g. ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1).
The script launches only uwb_eskf_node, publishes synthetic FRD IMU/quaternions
and calibrated ranges, then stops its own child. --imu-gap pauses IMU at 15 s
for 0.5 s. JSON results/config/node output are retained under --out.
Dependencies: numpy, rclpy and the standard ROS message packages.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

ANCHORS = np.array([(0,0,.48),(4.8,0,.48),(4.8,3.6,.48),(0,3.6,.48),
                    (0,0,2.208),(4.8,0,2.208),(4.8,3.6,2.208),(0,3.6,2.208)],float)
LEVER = np.array([-.09,0.,.31])


def truth(t):
    """Body position/acceleration and level yaw; analytic smooth-start circle."""
    u=max(0.,t-2.)
    phase=.35*(u-1+math.exp(-u))
    rate=.35*(1-math.exp(-u)) if t>=2 else 0.
    angular_acc=.35*math.exp(-u) if t>=2 else 0.
    p=np.array([2.4+.6*math.cos(phase),1.8+.6*math.sin(phase),.8])
    acc=np.array([-.6*(math.cos(phase)*rate*rate+math.sin(phase)*angular_acc),
                   .6*(-math.sin(phase)*rate*rate+math.cos(phase)*angular_acc),0.])
    yaw=.2+.03*u
    c,s=math.cos(yaw),math.sin(yaw)
    R=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    return p,acc,R,.03 if t>=2 else 0.


def run(args):
    # ROS imports stay lazy so --help and truth validation work on the desktop.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from geometry_msgs.msg import QuaternionStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,Range
    from std_msgs.msg import String

    # This tool is intentionally isolated from any active onboard recording node.
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or int(os.environ.get('ROS_DOMAIN_ID','0'))==0:
        raise ValueError('Set ROS_LOCALHOST_ONLY=1 and a dedicated nonzero ROS_DOMAIN_ID, e.g. 91')
    out=args.out.expanduser().resolve()
    out.mkdir(parents=True,exist_ok=True)
    rig=out/'smoke_rig.txt'
    rig.write_text(''.join(f'an{i+1}_p, {a[0]}, {a[1]}, {a[2]}\n' for i,a in enumerate(ANCHORS)))
    params=dict(imu='mavlink',imu_topic='/px4/imu_frd',attitude_topic='/px4/attitude_frd',imu_link='px4_imu_flu',
                imu_R=[1.,0.,0.,0.,-1.,0.,0.,0.,-1.],lever=LEVER.tolist(),yaw0_deg=math.degrees(.2),
                rig_file=str(rig),configuration_confirmed=True,config_sha256='synthetic-smoke-only',ignore_rig_bias=True,
                na=.45,ng=.016,nba=.003,nbg=.0001,imu_average_s=.02,min_anchor_interval_s=.02,use_px4_attitude=1,
                reinit_keep_attitude=True,att_sig_tilt_deg=2.,att_sig_yaw_deg=5.,att_gate=25.,att_min_interval_s=.02,
                imu_timeout_s=.2,range_timeout_s=.5,max_input_age_s=.25,min_init_anchors=6)
    config=out/'smoke.yaml'
    config.write_text(json.dumps({'uwb_eskf':{'ros__parameters':params}},indent=2))
    command=([str(args.node_exe.resolve())] if args.node_exe else ['ros2','run','uwb_eskf','uwb_eskf_node'])
    command+=['--ros-args','--params-file',str(config)]
    log=(out/'node.txt').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
    rclpy.init()

    class Smoke(Node):
        def __init__(self):
            super().__init__('eskf_synthetic_smoke')
            qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
            self.imu=self.create_publisher(Imu,'/px4/imu_frd',qos)
            self.att=self.create_publisher(QuaternionStamped,'/px4/attitude_frd',qos)
            self.ranges=[self.create_publisher(Range,f'/uwb/anchor_{i+1}/range',qos) for i in range(8)]
            self.create_subscription(Odometry,'/uwb_eskf/odom',self.odometry,qos)
            self.create_subscription(String,'/uwb_eskf/status',self.status,qos)
            self.start_ns=None
            self.start_wall=None
            self.first_running=None
            self.recovered=None
            self.errors=[]
            self.latest={}
            self.status_history=[]
            self.step=0
            self.late_samples=0
            self.rng=np.random.default_rng(83)
            self.acc_std=np.array([1.2,4.3,1.8])
            self.gyro_std=np.radians([9.,6.6,5.5])
            self.acc_phase=self.rng.uniform(0,2*math.pi,(2,3))
            self.gyro_phase=self.rng.uniform(0,2*math.pi,(2,3))

        def status(self,msg):
            self.latest=json.loads(msg.data)
            if self.start_wall is None:return
            elapsed=time.monotonic()-self.start_wall
            self.status_history.append(dict(t=elapsed,**self.latest))
            if self.latest['state']=='RUNNING':
                if self.first_running is None:self.first_running=elapsed
                if args.imu_gap and elapsed>=15.5 and self.latest.get('imu_gap_resets')==1 and self.recovered is None:
                    self.recovered=elapsed-15.5

        def odometry(self,msg):
            if self.start_ns is None:return
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            t=(stamp-self.start_ns)/1e9
            if t<=6 or t>30:return
            p=msg.pose.pose.position
            self.errors.append(float(np.linalg.norm(np.array([p.x,p.y,p.z])-truth(t)[0])))

        def send(self):
            t=self.step/200.
            p,acc,R,yaw_rate=truth(t)
            stamp_ns=self.start_ns+round(t*1e9)
            sec,nanosec=divmod(stamp_ns,1_000_000_000)
            def stamp(msg,frame):
                msg.header.stamp.sec=sec;msg.header.stamp.nanosec=nanosec;msg.header.frame_id=frame
            if not (args.imu_gap and 15.<=t<15.5):
                a=R.T@(acc+[0.,0.,9.81]);w=np.array([0.,0.,yaw_rate])
                # Half the variance in motor-like tones, half in white noise.
                a+=sum(.5**.5*self.acc_std*np.sin(2*math.pi*f*t+phase) for f,phase in zip((137.,274.),self.acc_phase))
                w+=sum(.5**.5*self.gyro_std*np.sin(2*math.pi*f*t+phase) for f,phase in zip((137.,274.),self.gyro_phase))
                a+=self.rng.normal(0,self.acc_std*math.sqrt(.5))
                w+=self.rng.normal(0,self.gyro_std*math.sqrt(.5))
                a*=np.array([1,-1,-1]);w*=np.array([1,-1,-1])
                msg=Imu();stamp(msg,'px4_imu_frd');msg.orientation_covariance[0]=-1.
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,a)
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,w)
                self.imu.publish(msg)
            if self.step%4==0:
                # Level FRD->NED: rig yaw offset=.2, so PX4 yaw=.2-rig_yaw.
                yaw=.2-math.atan2(R[1,0],R[0,0])
                msg=QuaternionStamped();stamp(msg,'px4_local_ned')
                msg.quaternion.w=math.cos(yaw/2);msg.quaternion.z=math.sin(yaw/2)
                self.att.publish(msg)
            if self.step%2==0:
                tag=p+R@LEVER
                for pub,anchor in zip(self.ranges,ANCHORS):
                    msg=Range();stamp(msg,'uwb_rig');msg.min_range=0.;msg.max_range=100.
                    msg.range=float(np.linalg.norm(tag-anchor)+self.rng.normal(0,.01));pub.publish(msg)
            self.step+=1

    node=None
    try:
        node=Smoke()
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Estimator exited; see node.txt')
            rclpy.spin_once(node,timeout_sec=.01)
            if all(p.get_subscription_count()>0 for p in [node.imu,node.att,*node.ranges]):break
        else:raise RuntimeError('Estimator subscriptions did not appear within 15 s')
        node.start_ns=node.get_clock().now().nanoseconds
        node.start_wall=time.monotonic()
        while node.step<6000:
            if process.poll() is not None:raise RuntimeError('Estimator exited during simulation')
            due=node.start_wall+node.step/200.
            now=time.monotonic()
            if now<due:
                rclpy.spin_once(node,timeout_sec=min(.002,due-now))
                continue
            if now-due>.05:node.late_samples+=1
            node.send()
            rclpy.spin_once(node,timeout_sec=0)
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
        rms=float(np.sqrt(np.mean(np.square(node.errors)))) if node.errors else None
        c=node.latest
        checks=dict(running_within_1s=node.first_running is not None and node.first_running<1.,
                    no_range_reinit=c.get('reinits')==0,position_rms_under_5cm=rms is not None and rms<.05,
                    enough_odometry=len(node.errors)>1000,attitude_updated=c.get('attitude_accepted',0)>0,
                    rate_limit_active=c.get('skipped_ranges',0)>0,
                    expected_imu_gap_resets=c.get('imu_gap_resets')==(1 if args.imu_gap else 0))
        if args.imu_gap:checks['recovered_within_0_5s']=node.recovered is not None and node.recovered<.5
        result=dict(passed=all(checks.values()),checks=checks,first_running_s=node.first_running,
                    recovery_delay_s=node.recovered,position_rms_m=rms,scored_odometry=len(node.errors),
                    late_samples_over_50ms=node.late_samples,final_status=c,imu_gap=args.imu_gap,
                    domain_id=os.environ['ROS_DOMAIN_ID'],scope='Synthetic localhost ROS only; no hardware or PX4 parameters accessed')
        (out/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        (out/'status_history.json').write_text(json.dumps(node.status_history,indent=2,allow_nan=False))
        print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=5)
        log.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--imu-gap',action='store_true')
    parser.add_argument('--node-exe',type=Path,help='Optional explicit binary; otherwise ros2 run uwb_eskf')
    args=parser.parse_args(argv)
    try:return run(args)
    except (ImportError,OSError,ValueError,RuntimeError) as exc:
        print(f'smoke_test_eskf: {exc}',file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
