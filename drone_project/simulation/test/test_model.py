import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools'/f'{name}.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


class ModelTest(unittest.TestCase):
    def test_mass_and_parallel_axis_inertia(self):
        config = json.loads((ROOT/'config/drone.json').read_text())
        mass, cg, tensor = load('prepare_model').mass_properties(config)
        self.assertAlmostEqual(mass + 4*0.003, 0.350)
        self.assertAlmostEqual(cg[0], 0); self.assertAlmostEqual(cg[1], 0)
        self.assertGreater(cg[2], 0.00925)
        for i in range(3):
            self.assertGreater(tensor[i][i], 0)
            self.assertLess(tensor[i][i], sum(tensor[j][j] for j in range(3) if j != i))
            for j in range(3): self.assertAlmostEqual(tensor[i][j], tensor[j][i])

    def test_native_parser_rejects_partial_and_accepts_boolean(self):
        parse = load('native_observer').parse_native
        parsed = parse('timestamp: 1234\narmed: True\nlockdown: 0\nkill: false\ntermination: 0\n')
        self.assertEqual(parsed['armed'], 1); self.assertEqual(parsed['lockdown'], 0)
        with self.assertRaises(ValueError): parse('armed: 1')

    def test_scenarios_preserve_threshold_and_no_release_failures(self):
        cases = json.loads((ROOT/'config/scenarios.json').read_text())
        self.assertEqual(len(cases), 8)
        self.assertGreater(cases['valid']['velocity_enu_m_s'][2], 5)
        self.assertLess(cases['below_threshold']['velocity_enu_m_s'][2], 5)
        for name in ['operator_abort','launch_timeout','command_rejection','safety_denial','communication_loss']:
            self.assertFalse(cases[name]['release'])

    def test_generated_motor_allocation_matches_flu_to_frd(self):
        import os
        px4 = Path(os.environ.get('PX4_ROOT', str(Path.home()/'PX4-Autopilot')))
        if not px4.exists(): self.skipTest('installed PX4 source unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            report = load('prepare_model').generate(px4, Path(tmp))
            model = ET.parse(Path(tmp)/'models/drone_phase6/model.sdf').getroot().find('model')
            env = (Path(tmp)/'px4.env').read_text()
            self.assertAlmostEqual(report['total_mass_kg'], 0.35)
            self.assertEqual(len(model.findall('joint')), 4)
            self.assertGreaterEqual(len(model.find("link[@name='base_link']").findall('sensor')), 3)
            self.assertIn('PX4_PARAM_CA_ROTOR0_PY=0.1', env)
            self.assertNotIn('COM_ARM_', env)
            self.assertNotIn('COM_RC_IN_MODE', env)


if __name__ == '__main__': unittest.main()
