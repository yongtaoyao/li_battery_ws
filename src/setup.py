from setuptools import setup
import os
from glob import glob

package_name = 'li_battery_diagnostic_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.xml')),
        ('share/' + package_name +'/config', glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('resource/*')),
        # Explicitly install the marker file for the package index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yongtao',
    maintainer_email='yongtaoyao001@gmail.com',
    description='Python-based Li Battery Diagnostic Module for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'battery_diagnostic_node = li_battery_diagnostic_module.battery_diagnostic_node:main'
    #     ],
    # },
    entry_points={
    'console_scripts': [
        'battery_diagnostic_node = li_battery_diagnostic_module.battery_diagnostic_node:main',
        'advanced_battery_diagnostic_node = li_battery_diagnostic_module.advanced_battery_diagnostic_node:main'
    ],
},

)
