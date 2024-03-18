from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ttl_usb_ros_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Szymon Kwiatkowski',
    maintainer_email='szymon.z.kwiatkowski@gmail.com',
    description='ROS 2 TTL GPS package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ttl_usb_node = ttl_usb_ros_pkg.ttl_usb_node:main'
        ],
    },
)
