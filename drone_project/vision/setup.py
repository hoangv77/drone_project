from glob import glob

from setuptools import setup

setup(
    name='drone_vision',
    version='0.1.0',
    packages=['drone_vision'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_vision']),
        ('share/drone_vision', ['package.xml']),
        ('share/drone_vision/config', glob('config/*.yaml')),
        ('share/drone_vision/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    maintainer='drone_project contributors',
    maintainer_email='noreply@github.com',
    description='Calibrated AprilTag perception and standalone flight recording',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'camera_gate = drone_vision.camera_gate:main',
        'test_camera = drone_vision.test_camera:main',
        'recorder = drone_vision.recorder:main',
    ]},
)
