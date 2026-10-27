#!/usr/bin/env python3
"""Explicit operator request + physical release + timestamped ROS metrics."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double
from px4_interface_node.msg import DroneState
from px4_interface_node.srv import ExecuteCommand
from px4_msgs.msg import VehicleAttitude
from safety_monitor.msg import SafetyStatus
from state_machine_manager.msg import MissionState as M, NativeLaunchStatus
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='valid')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = json.loads((Path(__file__).resolve().parents[1]/'config/scenarios.json').read_text())
    case = cases[args.scenario]
    if args.scenario in ['command_rejection','communication_loss','safety_denial']:
        raise SystemExit('Use isolated ROS contract tests for controlled fault injection; see README')
    rclpy.init(); node = rclpy.create_node('explicit_sitl_throw_test')
    data = {}; records = []; started = time.monotonic(); release_at = None
    drone_sequence = 0
    native_detection = None; stabilized_at = None; release_sim_t = None; ground_stable_at = None
    truth_records = []
    gz_node = GzNode()
    def truth(message):
        for pose in message.pose:
            if pose.name == 'drone_phase6_0':
                q=pose.orientation
                truth_records.append({'t':time.monotonic()-started,
                    'sim_t':message.header.stamp.sec+message.header.stamp.nsec*1e-9,
                    'x':pose.position.x,'y':pose.position.y,'z':pose.position.z,
                    'tilt_rad':math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y))))})
    gz_node.subscribe(Pose_V,'/world/throw_world/pose/info',truth)
    def released(message):
        nonlocal release_at, release_sim_t
        release_at=time.monotonic(); release_sim_t=message.data
    gz_node.subscribe(Double,'/phase6/released',released)
    def truth_stable():
        samples=list(truth_records)
        if len(samples)<3 or samples[-1]['t'] < time.monotonic()-started-0.3: return False
        recent=[s for s in samples if s['sim_t'] >= samples[-1]['sim_t']-1.1]
        if recent[-1]['sim_t']-recent[0]['sim_t']<1.0: return False
        for before,after in zip(recent,recent[1:]):
            dt=after['sim_t']-before['sim_t']
            if dt<=0: continue
            speed=math.sqrt(sum((after[k]-before[k])**2 for k in ['x','y','z']))/dt
            if speed>=0.7 or after['tilt_rad']>=0.17 or after['z']<=0.1: return False
        return True
    def record(key, message):
        nonlocal native_detection, stabilized_at, drone_sequence
        data[key] = message
        t = time.monotonic()
        if key == 'drone':
            drone_sequence += 1
            records.append({'t':t-started,'z_ned':message.z,'landed':message.landed,'armed':message.armed})
        if key == 'native' and release_at is not None and message.armed and not message.lockdown and native_detection is None:
            native_detection = t
        if key == 'mission':
            if message.current_state == M.STANDBY and message.mission_result == M.RESULT_SUCCEEDED:
                stabilized_at = t
        if key == 'attitude':
            q = message.q; tilt = math.acos(max(-1,min(1,1-2*(q[1]**2+q[2]**2))))
            records.append({'t':t-started, 'tilt_rad':tilt})
    subscriptions = []
    for key, kind, topic in [('drone',DroneState,'/drone/state'), ('safety',SafetyStatus,'/drone/safety_status'),
                              ('mission',M,'/drone/mission_state'),('native',NativeLaunchStatus,'/simulation/native_launch_status'),
                              ('attitude',VehicleAttitude,'/fmu/out/vehicle_attitude')]:
        qos = rclpy.qos.qos_profile_sensor_data if key == 'attitude' else 10
        subscriptions.append(node.create_subscription(kind, topic, lambda m,k=key: record(k,m), qos))
    def until(predicate, timeout):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('SITL readiness/outcome deadline: '+str({k:str(v) for k,v in data.items() if k in ['safety','mission']}))
            rclpy.spin_once(node, timeout_sec=0.02)
    def call(client, request):
        if not client.wait_for_service(timeout_sec=10): raise TimeoutError('Service unavailable')
        future=client.call_async(request); until(future.done,10); return future.result()
    outcome = 'error'; detail = ''
    try:
        command=node.create_client(ExecuteCommand,'/drone/execute_command')
        until(lambda:'drone' in data and data['drone'].status_fresh,90)
        if data['drone'].flight_mode != 'LOITER':
            request=ExecuteCommand.Request(); request.command=request.SET_MODE; request.mode='LOITER'
            result=call(command,request)
            if not result.success: raise RuntimeError(result.message)
        after_mode = drone_sequence
        until(lambda: truth_records and drone_sequence >= after_mode+2 and data['drone'].ready_to_arm and
              not data['drone'].command_pending and 'safety' in data and
              data['safety'].launch_permitted and 'native' in data,60)
        start=node.create_client(Trigger,'/drone/mission/start')
        result=call(start,Trigger.Request())
        if not result.success: raise RuntimeError(result.message)
        until(lambda: data.get('mission') and data['mission'].current_state == M.ARMED_WAITING,15)
        if args.scenario == 'operator_abort':
            call(node.create_client(Trigger,'/drone/mission/abort'),Trigger.Request())
        elif case.get('release',True):
            release=subprocess.run(['gz','service','-s','/phase6/release','--reqtype','gz.msgs.Boolean',
                '--reptype','gz.msgs.Boolean','--timeout','2000','--req','data: true'],capture_output=True,text=True,timeout=4)
            if release.returncode or 'data: true' not in release.stdout: raise RuntimeError('Simulator release failed: '+release.stdout+release.stderr)
            until(lambda:release_at is not None,6)
        until(lambda: data['mission'].current_state in [M.ABORTED,M.FAULT] or stabilized_at is not None,70)
        state=data['mission']
        outcome='stabilized' if stabilized_at else state.state_name.lower()
        detail=state.transition_reason
        if outcome=='stabilized':
            until(truth_stable,8)
            ground_stable_at=time.monotonic()
            post=[s for s in list(truth_records) if s['t']>=release_at-started]
            pre=[s for s in list(truth_records) if s['t']<=release_at-started]
            if args.scenario=='valid' and (not pre or max(s['z'] for s in post)<pre[-1]['z']+0.5):
                raise RuntimeError('Ground truth did not show the expected upward throw trajectory')
            if args.scenario=='valid' and min(s['z'] for s in post)<0.05:
                raise RuntimeError('Ground contact before stabilization invalidates a successful throw')
    except Exception as error:
        outcome='error'; detail=str(error)
    finally:
        release_t=None if release_at is None else release_at-started
        heights=[-r['z_ned'] for r in records if 'z_ned' in r and release_t is not None and r['t']>=release_t]
        release_samples=[-r['z_ned'] for r in records if 'z_ned' in r and release_t is not None and r['t']<=release_t]
        initial=release_samples[-1] if release_samples else None
        truth_after=[s for s in list(truth_records) if release_t is not None and s['t']>=release_t]
        truth_before=[s for s in list(truth_records) if release_t is not None and s['t']<=release_t]
        true_initial=None if not truth_before else truth_before[-1]['z']
        report={'scenario':args.scenario,'outcome':outcome,'detail':detail,
            'final_armed':None if 'drone' not in data else data['drone'].armed,
            'ground_truth_stable':truth_stable(),
            'ground_truth_peak_altitude_m':max([s['z'] for s in truth_after],default=None),
            'release_sim_time_s':release_sim_t,
            'native_detected':native_detection is not None,
            'release_to_detection_s':None if native_detection is None else native_detection-release_at,
            'release_to_stable_s':None if stabilized_at is None or release_at is None else stabilized_at-release_at,
            'release_to_ground_truth_stable_s':None if ground_stable_at is None or release_at is None else ground_stable_at-release_at,
            'maximum_altitude_loss_m':None if not truth_after or true_initial is None else max(0,true_initial-min(s['z'] for s in truth_after)),
            'estimator_altitude_loss_m':None if not heights or initial is None else max(0,initial-min(heights)),
            'maximum_tilt_rad':max([s['tilt_rad'] for s in truth_after],default=None),
            'records':records,'ground_truth_records':list(truth_records),
            'limitations':'Detection timing uses ROS receipt time; altitude/tilt use independent Gazebo pose; model propulsion/mass provisional'}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ['records','ground_truth_records']},indent=2))
        gz_node.unsubscribe('/world/throw_world/pose/info')
        gz_node.unsubscribe('/phase6/released')
        node.destroy_node();rclpy.shutdown()
    expected = (outcome=='stabilized' if args.scenario=='valid' else
                outcome=='aborted' and report['final_armed'] is False if args.scenario in ['operator_abort','launch_timeout'] else
                release_at is not None and outcome!='error' and not report['native_detected'] if args.scenario=='below_threshold' else outcome!='error')
    raise SystemExit(0 if expected else 1)


if __name__=='__main__': main()
