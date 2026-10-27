#!/usr/bin/env python3
"""Generate standalone SDF and a parameter environment from audited inputs."""
import argparse
import hashlib
import json
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def mass_properties(config):
    a = config['assumptions']
    # Each tuple is (mass, COM, box dimensions), all provisional except frame size.
    components = [(a['frame_mass_kg'], [0, 0, 0.00925], config['cad']['size_m']),
                  (a['electronics_mass_kg'], [0, 0, 0.01], [0.035, 0.035, 0.012]),
                  (a['battery_mass_kg'], a['battery_position_m'], a['battery_size_m'])]
    for position in a['motor_positions_flu_m']:
        components.append((a['motor_mass_kg_each'], position, [0.014, 0.014, 0.016]))
    mass = sum(c[0] for c in components)
    cg = [sum(m * p[i] for m, p, _ in components) / mass for i in range(3)]
    tensor = [[0.0] * 3 for _ in range(3)]
    for m, p, dims in components:
        r = [p[i] - cg[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                tensor[i][j] += m * (sum(x*x for x in r) - r[i]*r[i]) if i == j else -m*r[i]*r[j]
            tensor[i][i] += m * sum(dims[j]**2 for j in range(3) if j != i) / 12
    return mass, cg, tensor


def tag(parent, element_name, value=None, **attrs):
    child = ET.SubElement(parent, element_name, attrs)
    if value is not None:
        child.text = ' '.join(map(str, value)) if isinstance(value, (list, tuple)) else str(value)
    return child


def inertia(link, mass, cg, tensor):
    inertial = tag(link, 'inertial')
    tag(inertial, 'mass', mass)
    tag(inertial, 'pose', [*cg, 0, 0, 0])
    block = tag(inertial, 'inertia')
    for key, i, j in [('ixx',0,0), ('iyy',1,1), ('izz',2,2), ('ixy',0,1), ('ixz',0,2), ('iyz',1,2)]:
        tag(block, key, tensor[i][j])


def generate(px4, output, scenario='valid'):
    config = json.loads((ROOT / 'config/drone.json').read_text())
    case = json.loads((ROOT / 'config/scenarios.json').read_text())[scenario]
    a = config['assumptions']
    mass, cg, tensor = mass_properties(config)
    directory = output / 'models' / 'drone_phase6'
    directory.mkdir(parents=True, exist_ok=True)
    cad_mesh = ROOT/'models/drone_phase6/meshes/frame.stl'
    if cad_mesh.exists():
        (directory/'meshes').mkdir(exist_ok=True)
        target=directory/'meshes/frame.stl'
        if cad_mesh.resolve() != target.resolve(): shutil.copy2(cad_mesh,target)
    sdf = ET.Element('sdf', version='1.9')
    model = tag(sdf, 'model', name='drone_phase6')
    tag(model, 'pose', [0, 0, case['height_m'], 0, 0, 0])
    base = tag(model, 'link', name='base_link')
    inertia(base, mass, cg, tensor)
    for kind in ['visual', 'collision']:
        body = tag(base, kind, name='cad_envelope_' + kind)
        if kind == 'visual' and cad_mesh.exists():
            tag(tag(tag(body, 'geometry'), 'mesh'), 'uri', 'model://drone_phase6/meshes/frame.stl')
        else:
            tag(body, 'pose', [0,0,0.00925,0,0,0])
            tag(tag(tag(body, 'geometry'), 'box'), 'size', config['cad']['size_m'])
    visual = tag(base, 'visual', name='battery_assumed')
    tag(visual, 'pose', [*a['battery_position_m'],0,0,0])
    tag(tag(tag(visual, 'geometry'), 'box'), 'size', a['battery_size_m'])
    # Sensor schemas and names come from the installed, revision-pinned PX4
    # model so the local gz_bridge finds the same IMU/barometer/GNSS topics.
    source = px4 / 'Tools/simulation/gz/models/x500_base/model.sdf'
    reference = ET.parse(source).getroot().find('model/link')
    for sensor in reference.findall('sensor'):
        pose = sensor.find('pose')
        if pose is None:
            pose = ET.SubElement(sensor, 'pose')
        pose.text = ' '.join(map(str,[*a['sensor_pose_flu_m'],0,0,0]))
        base.append(sensor)
    env = {'PX4_SYS_AUTOSTART': '4001', 'PX4_SIM_MODEL': 'gz_drone_phase6',
           'PX4_GZ_WORLD': 'throw_world', 'PX4_PARAM_COM_THROW_EN': '1',
           'PX4_PARAM_COM_THROW_SPEED': '5.0'}
    for i, p in enumerate(a['motor_positions_flu_m']):
        rotor = tag(model, 'link', name=f'rotor_{i}')
        tag(rotor, 'pose', [*p,0,0,0])
        m = a['prop_mass_kg_each']; radius = a['prop_diameter_m']/2
        inertia(rotor, m, [0,0,0], [[m*radius**2/4,0,0],[0,m*radius**2/4,0],[0,0,m*radius**2/2]])
        v = tag(rotor, 'visual', name=f'prop_{i}')
        cylinder = tag(tag(v, 'geometry'), 'cylinder')
        tag(cylinder, 'radius', radius); tag(cylinder, 'length', 0.001)
        joint = tag(model, 'joint', name=f'rotor_{i}_joint', type='revolute')
        tag(joint, 'parent', 'base_link'); tag(joint, 'child', f'rotor_{i}')
        tag(tag(joint, 'axis'), 'xyz', [0,0,1])
        plugin = tag(model, 'plugin', filename='gz-sim-multicopter-motor-model-system',
                     name='gz::sim::systems::MulticopterMotorModel')
        values = {'jointName': f'rotor_{i}_joint', 'linkName': f'rotor_{i}',
                  'turningDirection': 'ccw' if i < 2 else 'cw', 'motorNumber': i,
                  'commandSubTopic': 'command/motor_speed', 'motorType': 'velocity',
                  'rotorVelocitySlowdownSim': 10, 'motorConstant': a['motor_constant_N_s2_rad2'],
                  'momentConstant': a['moment_constant_m'], 'maxRotVelocity': a['max_rot_velocity_rad_s'],
                  'timeConstantUp': a['time_constant_up_s'], 'timeConstantDown': a['time_constant_down_s'],
                  'rotorDragCoefficient': a['rotor_drag_coefficient'], 'rollingMomentCoefficient': 0}
        for key, value in values.items(): tag(plugin, key, value)
        # PX4 allocation axes are FRD, Gazebo model axes are FLU.
        for axis, value in zip('XYZ', [p[0], -p[1], -p[2]]):
            env[f'PX4_PARAM_CA_ROTOR{i}_P{axis}'] = str(value)
        env[f'PX4_PARAM_CA_ROTOR{i}_KM'] = str(a['moment_constant_m'] * (1 if i < 2 else -1))
        env[f'PX4_PARAM_SIM_GZ_EC_MIN{i+1}'] = '0'
    total_mass = mass + 4*a['prop_mass_kg_each']
    hover = math.sqrt(total_mass*9.80665 / (4*a['motor_constant_N_s2_rad2'])) / a['max_rot_velocity_rad_s']
    env['PX4_PARAM_MPC_THR_HOVER'] = str(hover)
    release = tag(model, 'plugin', filename='libThrowRelease.so', name='ThrowRelease')
    tag(release, 'hold_pose', [0,0,case['height_m'],0,0,0])
    tag(release, 'release_rpy', case['rpy_rad'])
    tag(release, 'release_velocity', case['velocity_enu_m_s'])
    tag(release, 'push_duration', 0.2)
    ET.indent(sdf); ET.ElementTree(sdf).write(directory/'model.sdf', encoding='utf-8', xml_declaration=True)
    (directory/'model.config').write_text('<model><name>drone_phase6</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>\n')
    worlds = output/'worlds'; worlds.mkdir(exist_ok=True)
    world = ET.parse(px4/'Tools/simulation/gz/worlds/default.sdf')
    w = world.getroot().find('world'); w.set('name', 'throw_world')
    pedestal = tag(w, 'model', name='hand_pedestal')
    tag(pedestal, 'static', 'true')
    height = case['height_m'] - 0.0035
    tag(pedestal, 'pose', [0,0,height/2,0,0,0])
    support = tag(pedestal, 'link', name='support')
    for kind in ['visual', 'collision']:
        tag(tag(tag(tag(support, kind, name=kind), 'geometry'), 'box'), 'size', [0.02,0.02,height])
    # Explicit physics and sensor systems allow server-only operation.
    for filename, name in [('physics','Physics'), ('user-commands','UserCommands'),
                           ('scene-broadcaster','SceneBroadcaster'), ('imu','Imu'),
                           ('air-pressure','AirPressure'), ('navsat','NavSat'), ('magnetometer','Magnetometer')]:
        plugin = tag(w, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Magnetometer':
            tag(plugin, 'use_units_gauss', 'false')
            tag(plugin, 'use_earth_frame_ned', 'false')
    ET.indent(world); world.write(worlds/'throw_world.sdf', encoding='utf-8', xml_declaration=True)
    output.mkdir(exist_ok=True)
    (output/'px4.env').write_text(''.join(f'export {k}={v}\n' for k,v in env.items()))
    report = {'base_mass_kg':mass, 'total_mass_kg':total_mass, 'base_cg_m':cg,
              'base_inertia_kg_m2':tensor, 'hover_throttle_estimate':hover,
              'impulse_N_s': [total_mass*v for v in case['velocity_enu_m_s']],
              'push_duration_s': 0.2,
              'average_hand_force_N': [total_mass*v/0.2 + (total_mass*9.8 if i==2 else 0)
                                       for i,v in enumerate(case['velocity_enu_m_s'])],
              'scenario':scenario, 'assumptions':a,
              'sensor_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    (output/'model_report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', default='valid')
    args = parser.parse_args()
    print(json.dumps(generate(args.px4.resolve(), args.output.resolve(), args.scenario), indent=2))
