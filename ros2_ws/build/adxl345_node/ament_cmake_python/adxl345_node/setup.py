from setuptools import find_packages
from setuptools import setup

setup(
    name='adxl345_node',
    version='0.1.0',
    packages=find_packages(
        include=('adxl345_node', 'adxl345_node.*')),
)
