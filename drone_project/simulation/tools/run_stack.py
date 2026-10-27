#!/usr/bin/env python3
"""Start the full foreground SITL stack; Ctrl-C stops only its child groups."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='valid')
    parser.add_argument('--test', action='store_true', help='Explicit permission to request launch and run selected test')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    tools = Path(__file__).resolve().parent
    px4 = Path(os.environ.get('PX4_ROOT', str(Path.home()/'PX4-Autopilot'))).resolve()
    run = repo/'.phase6/sim'; run.mkdir(parents=True, exist_ok=True)
    # Runtime state must be fresh per scenario; never reuse persisted settings.
    trial=args.scenario+'-'+time.strftime('%Y%m%d-%H%M%S')
    runtime = run/('runtime-'+trial)
    env = dict(os.environ, PHASE6_RUN_DIR=str(runtime.parent/runtime.name))
    agent = os.environ.get('MICRO_XRCE_AGENT', str(Path.home()/'Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'))
    if subprocess.run(['pgrep','-x','px4'], capture_output=True).returncode == 0:
        raise SystemExit('A PX4 process is already running; stop it before starting an isolated scenario')
    children = []; handles = []; launch_pids = set()
    def spawn(label, command, child_env=None):
        handle = (run/(trial+'-'+label+'.log')).open('w'); handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   env=child_env or env, start_new_session=True)
        children.append(process)
        if label=='ros': launch_pids.add(process.pid)
        return process
    try:
        spawn('agent', [agent,'udp4','-p','8888'])
        spawn('sitl', ['bash',str(tools/'run_sitl.sh'),args.scenario])
        spawn('operator', ['python3',str(tools/'operator_link.py')])
        spawn('ros', ['ros2','launch','state_machine_manager','throw_sitl.launch.py',
                      'px4_bin:='+str(px4/'build/px4_sitl_default/bin'),
                      'observer_script:='+str(tools/'native_observer.py')])
        if args.test:
            result = subprocess.run(['python3',str(tools/'run_scenario.py'),'--scenario',args.scenario,
                                     '--output',str(run/(trial+'-metrics.json'))], env=env)
            return result.returncode
        print('SITL stack started. Launch requires /drone/mission/start or a separate run_scenario.py request.', flush=True)
        while True:
            for child in children:
                if child.poll() is not None: raise RuntimeError('Stack process exited; inspect '+str(run))
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    finally:
        for child in reversed(children):
            if child.poll() is None:
                # ROS launch forwards the signal to its own children.
                if child.pid in launch_pids: child.send_signal(signal.SIGINT)
                else: os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGTERM)
        for handle in handles: handle.close()


if __name__ == '__main__': raise SystemExit(main())
