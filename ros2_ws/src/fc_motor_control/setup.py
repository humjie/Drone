from setuptools import find_packages, setup

package_name = 'fc_motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='mingjiehu5@gmail.com',
    description='To bridge between ROS2 and flight controller to receive the value from controller and control the motors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fc_motor_control = fc_motor_control.fc_motor_control:main',
        ],
    },
)
