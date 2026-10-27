#!/usr/bin/env python3
"""Independent ballistic regression: no PX4 or ROS telemetry in this test."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from prepare_model import generate


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--px4',type=Path,required=True)
    parser.add_argument('--plugin-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    os.environ['GZ_PARTITION']='phase6-ballistic-'+str(os.getpid())
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    samples=[]
    def observe(message):
        for pose in message.pose:
            if pose.name=='drone_phase6_0':
                samples.append((message.header.stamp.sec+message.header.stamp.nsec*1e-9,pose.position.z))
    with tempfile.TemporaryDirectory(prefix='phase6-ballistic-') as tmp:
        run=Path(tmp);generate(args.px4,run)
        world=ET.parse(run/'worlds/throw_world.sdf')
        include=ET.SubElement(world.getroot().find('world'),'include')
        ET.SubElement(include,'uri').text='model://drone_phase6'
        ET.SubElement(include,'name').text='drone_phase6_0'
        world.write(run/'ballistic.sdf')
        env=dict(os.environ,GZ_SIM_RESOURCE_PATH=str(run/'models'),GZ_SIM_SYSTEM_PLUGIN_PATH=str(args.plugin_dir.resolve()))
        log=(run/'gz.log').open('w')
        process=subprocess.Popen(['gz','sim','-s','-r',str(run/'ballistic.sdf')],env=env,
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node=Node();node.subscribe(Pose_V,'/world/throw_world/pose/info',observe)
        try:
            deadline=time.monotonic()+30
            while not samples:
                if time.monotonic()>deadline or process.poll() is not None: raise RuntimeError('Ballistic world unavailable')
                time.sleep(0.02)
            initial=samples[-1]; boundary=len(samples)
            released=subprocess.run(['gz','service','-s','/phase6/release','--reqtype','gz.msgs.Boolean',
                '--reptype','gz.msgs.Boolean','--timeout','2000','--req','data: true'],env=env,capture_output=True,text=True,timeout=4)
            if 'data: true' not in released.stdout: raise RuntimeError('Release service failed')
            deadline=time.monotonic()+10
            while samples[-1][0]-initial[0]<1.5:
                if time.monotonic()>deadline: raise RuntimeError('No advancing simulation poses')
                time.sleep(0.02)
            flight=samples[boundary:]
            peak=max(flight,key=lambda p:p[1])
            # 0.2 s hand push adds ~0.65 m, then v^2/(2g)=2.1556 m.
            passed=3.8<peak[1]<4.3 and peak[1]-flight[-1][1]>0.8 and 0.7<peak[0]-initial[0]<1.2
            result={'passed':passed,'peak_altitude_m':peak[1],
                    'request_to_apex_sim_s':peak[0]-initial[0],
                    'descent_after_apex_m':peak[1]-flight[-1][1],
                    'samples':len(flight),'px4_connected':False}
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
            if not passed: raise RuntimeError('Ballistic trajectory does not match the configured throw')
        finally:
            node.unsubscribe('/world/throw_world/pose/info')
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGTERM)
            log.close()


if __name__=='__main__':main()
