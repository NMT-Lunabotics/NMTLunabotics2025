from setuptools import setup

package_name = 'skid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Runs dig-dump cycle for robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dig_dump_service = skid_control.dig_dump_service:main',
        ],
    },
)
