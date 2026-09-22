import os
from glob import glob

from setuptools import setup

package_name = 'servo_sim_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sampreeth',
    maintainer_email='rk.sampreeth@gmail.com',
    description='Engine-neutral sim bridge driving the R2D3 neck via the real '
                'servo contract, shared by the MuJoCo and Gazebo simulations.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neck_servo_bridge = servo_sim_bridge.neck_servo_bridge:main',
        ],
    },
)
