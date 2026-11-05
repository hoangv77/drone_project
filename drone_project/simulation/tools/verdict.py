"""Flight-safety acceptance for completed SITL trials."""
from math import pi

EXPECTED = {
    'valid': ('stabilized', True, True),
    'below_threshold': ('aborted', True, False),
    'unfavorable_orientation': ('aborted', True, True),
    'launch_timeout': ('aborted', False, False),
    'operator_abort': ('aborted', False, False),
    'expected_rejection': ('rejected', False, False),
}


def evaluate(report):
    if not isinstance(report, dict):
        return dict(executed=False, behavior_passed=False, safety_passed=False,
                    passed=False, reason='Missing or malformed metrics')
    case = report.get('scenario')
    expected = EXPECTED.get(case) if isinstance(case, str) else None
    executed = (expected is not None and report.get('execution_ok') is True
                and report.get('outcome') in ('stabilized', 'aborted', 'fault', 'rejected'))
    behavior = safety = False
    if expected is not None:
        outcome, release_required, native_required = expected
        release = report.get('release_sim_time_s')
        release_observed = type(release) in (int, float) and 0 <= release < float('inf')
        if release_required:
            release_matches = release_observed
        else:
            release_matches = release is None and 'release_sim_time_s' in report
        behavior = (report.get('outcome') == outcome and release_matches
                    and report.get('native_detected') is native_required)
        if case == 'unfavorable_orientation':
            tilt = report.get('maximum_tilt_rad')
            behavior = behavior and type(tilt) in (int, float) and 2.5 <= tilt <= pi
        evidence = (report.get('terminal_telemetry_fresh') is True
                    and report.get('prohibited_impact') is False)
        if case == 'valid':
            safety = (evidence and report.get('outcome') == 'stabilized'
                      and report.get('ground_truth_stable') is True
                      and report.get('trajectory_verified') is True
                      and report.get('final_armed') is True
                      and report.get('final_landed') is False
                      and report.get('final_mode') == 'LOITER')
        else:
            safety = (evidence and report.get('final_armed') is False
                      and report.get('final_landed') is True
                      and report.get('outcome') in ('aborted', 'rejected'))
    return dict(executed=executed, behavior_passed=behavior, safety_passed=safety,
                passed=executed and behavior and safety,
                reason=('Execution incomplete' if not executed else
                        'Terminal safety not verified' if not safety else
                        'Unexpected scenario behavior' if not behavior else 'Accepted'))


def suite_record(case, returncode, metrics):
    valid = isinstance(metrics, dict) and metrics.get('scenario') == case
    report = metrics.copy() if valid else {'scenario': case}
    verdict = evaluate(report)
    # Preserve the process exit code separately. Neither source can override a failure.
    report.update(returncode=returncode, verdict=verdict,
                  passed=valid and returncode == 0 and verdict['passed'])
    return report
