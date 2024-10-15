from setuptools import setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install the marker
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        
        # Install the launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install the configuration file
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Include package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of the motor_control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control.motor_control_node:main',  # Make sure this line exists
        ],
    },
)
