#!/usr/bin/env python3
"""Repeat independent SITL worlds and collect only observed trial results."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from verdict import suite_record


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cases',nargs='+',default=['valid','valid','below_threshold','unfavorable_orientation','launch_timeout','operator_abort'])
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();results=[]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for index,case in enumerate(args.cases):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'metrics.json'
            result = subprocess.run([sys.executable, str(Path(__file__).with_name('run_stack.py')),
                                     '--scenario', case, '--test', '--metrics', str(path)],
                                    capture_output=True, text=True)
            try:
                metrics = json.loads(path.read_text())
            except (OSError, ValueError):
                metrics = None
        record = suite_record(case, result.returncode, metrics)
        if metrics is None:
            record['detail'] = result.stdout + result.stderr
        results.append(record)
        valid=[r for r in results if r['scenario']=='valid']
        report={'trials':results,'nominal_attempts':len(valid),
                'nominal_successes':sum(r['passed'] for r in valid)}
        report['nominal_success_rate']=report['nominal_successes']/len(valid) if valid else None
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(f'{index+1}/{len(args.cases)} {case}: exit={result.returncode} outcome={record.get("outcome","error")}',flush=True)
    return int(any(not r['passed'] for r in results))


if __name__=='__main__':raise SystemExit(main())
