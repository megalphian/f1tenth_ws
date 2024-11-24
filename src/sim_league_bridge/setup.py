from setuptools import setup
import os
from glob import glob

package_name = 'sim_league_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'racelines'), glob('racelines/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Megnath Ramesh',
    maintainer_email='m5ramesh@uwaterloo.ca',
    description='sim league bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_odom = sim_league_bridge.vehicle_odom:main',
            'vehicle_controller = sim_league_bridge.vehicle_controller:main'
        ],
    },
)
