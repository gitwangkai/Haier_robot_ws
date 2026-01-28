#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from setuptools import setup

package_name = 'robot_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aidlux',
    maintainer_email='chendongfang@aidlux.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follower = robot_mover.mover_node:main',
            'keyboard_teleop = robot_mover.keyboard_teleop:main',
            'test_rotation = robot_mover.test_rotation:main',
        ],
    },
)