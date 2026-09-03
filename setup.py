from glob import glob

from setuptools import find_packages, setup

package_name = 'crazyflie_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='khawaja',
    maintainer_email='khawaja.alamdar11@gmail.com',
    description=('Helper and stress-test tooling for Crazyflie UAV fleets on '
                 'ROS 2 (crazyswarm2)'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stress_test = crazyflie_utils.stress_test_node:main',
            'link_monitor = crazyflie_utils.link_monitor_node:main',
        ],
    },
)
