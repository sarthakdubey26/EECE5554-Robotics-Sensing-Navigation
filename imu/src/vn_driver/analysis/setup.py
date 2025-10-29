from setuptools import setup
import os
from glob import glob

package_name = 'vn_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sarthak Dubey',
    maintainer_email='dubey.sart@northeastern.edu',
    description='VectorNav IMU driver for EECE 5554 Lab 3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vn_driver = vn_driver.vn_driver:main',
        ],
    },
)
