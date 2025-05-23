from setuptools import setup
import os
from glob import glob

package_name = 'moon_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))) 
    ],

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ndev',
    maintainer_email='niallcdevlin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = moon_serial.serial_bridge_node:main',
            'serial_convert_node = moon_serial.serial_convert_node:main',
            'heartbeat_publisher = moon_serial.heartbeat_publisher:main',
        ],
    },
)
