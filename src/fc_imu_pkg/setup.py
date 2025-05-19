from setuptools import find_packages, setup

package_name = 'fc_imu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@todo.todo',
    description='Package for reading IMU data from F4 flight controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fc_imu_node = fc_imu_pkg.fc_imu_node:main'
        ],
    },
)
