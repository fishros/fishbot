from setuptools import setup

package_name = 'fishbot_controller'
submodules = [package_name+'/driver/base/',package_name+'/driver/laser/',package_name+'/tool/']

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name]+submodules,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='sangxin2014@sina.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller=fishbot_controller.controller:main'
        ],
    },
)
