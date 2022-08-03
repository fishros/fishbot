#!/usr/bin/env python

from setuptools import setup

package_name = 'rqt_tf_tree'
setup(
    name=package_name,
    version='1.0.2',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/RosTfTree.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Thibault Kruse',
    maintainer='Isaac I.Y. Saito',
    maintainer_email='gm130s@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rqt_tf_tree provides a GUI plugin for visualizing the ROS TF frame tree.'
    ),
    license='BSD',
    entry_points={
        'console_scripts': [
            'rqt_tf_tree = ' + package_name + '.main:main'
        ],
    },
)
