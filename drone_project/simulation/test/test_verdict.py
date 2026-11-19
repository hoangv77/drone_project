from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from verdict import evaluate, suite_record


class VerdictTests(unittest.TestCase):
    def test_nominal(self):
        self.assertTrue(evaluate(dict(scenario='valid', execution_ok=True, outcome='stabilized',
                                     release_sim_time_s=2.0, native_detected=True, final_armed=True,
                                     final_landed=False, final_mode='LOITER', ground_truth_stable=True,
                                     trajectory_verified=True, prohibited_impact=False,
                                     terminal_telemetry_fresh=True))['passed'])

    def test_recovery_and_rejection(self):
        for case, outcome, release in [('expected_rejection', 'rejected', None),
                                       ('below_threshold', 'aborted', 2.0),
                                       ('operator_abort', 'aborted', None)]:
            report = dict(scenario=case, execution_ok=True, outcome=outcome, native_detected=False,
                          release_sim_time_s=release, final_armed=False, final_landed=True,
                          prohibited_impact=False, terminal_telemetry_fresh=True)
            self.assertTrue(evaluate(report)['passed'])
            for fault in ({'outcome': 'fault', 'final_armed': True}, {'prohibited_impact': True},
                          {'terminal_telemetry_fresh': None}, {'final_landed': False},
                          {'execution_ok': False}, {'release_sim_time_s': 'invalid'}):
                with self.subTest(case=case, fault=fault):
                    self.assertFalse(evaluate(dict(report, **fault))['passed'])
            self.assertFalse(suite_record(case, 1, report)['passed'])

    def test_malformed_and_audited_outcomes_fail(self):
        for value in (None, [], {}, {'scenario': []}, {'scenario': 'valid', 'outcome': 'stabilized'}):
            self.assertFalse(evaluate(value)['passed'])
        for case in ('below_threshold', 'unfavorable_orientation', 'unfavorable_orientation'):
            report = dict(scenario=case, execution_ok=True, outcome='fault', final_armed=True,
                          final_landed=case == 'below_threshold', release_sim_time_s=2.0,
                          native_detected=case != 'below_threshold', terminal_telemetry_fresh=True,
                          prohibited_impact=case != 'below_threshold')
            self.assertFalse(suite_record(case, 0, report)['passed'])
