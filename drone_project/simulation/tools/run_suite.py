#!/usr/bin/env python3
"""Repeat independent SITL worlds and collect only observed trial results."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cases',nargs='+',default=['valid','valid','below_threshold','unfavorable_orientation','launch_timeout','operator_abort'])
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();results=[]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for index,case in enumerate(args.cases):
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('run_stack.py')),
                               '--scenario',case,'--test'],capture_output=True,text=True)
        record={'scenario':case,'returncode':result.returncode}
        try:
            start=result.stdout.index('{')
            metrics,_=json.JSONDecoder().raw_decode(result.stdout[start:])
            record.update(metrics)
        except (ValueError,json.JSONDecodeError): record['detail']=result.stdout+result.stderr
        results.append(record)
        valid=[r for r in results if r['scenario']=='valid']
        report={'trials':results,'nominal_attempts':len(valid),
                'nominal_successes':sum(r['returncode']==0 and r.get('outcome')=='stabilized' for r in valid)}
        report['nominal_success_rate']=report['nominal_successes']/len(valid) if valid else None
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(f'{index+1}/{len(args.cases)} {case}: exit={result.returncode} outcome={record.get("outcome","error")}',flush=True)
    return int(any(r['returncode']!=0 for r in results))


if __name__=='__main__':raise SystemExit(main())
