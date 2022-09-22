from setuptools import setup
from glob import glob
import os

package_name = 'fishbot_laser_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
        (os.path.join('share', package_name, 'launch'), glob('launch/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='87068644+fishros@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "laser_x2=fishbot_laser_driver.fishbot_laser_x2:main"
        ],
    },
)
