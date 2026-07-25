from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_uav_tf_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liumengyang',
    maintainer_email='user@example.com',
    description='multi uav tf manager',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tf_manager = multi_uav_tf_manager.tf_manager:main',
        ],
    },
)
