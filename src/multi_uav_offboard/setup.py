from setuptools import find_packages, setup

package_name = 'multi_uav_offboard'

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
    maintainer='liumengyang',
    maintainer_email='liumengyang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uav1_fastlio_mission = multi_uav_offboard.uav1_fastlio_mission:main',
            'uav1_fastlio_motion_test = multi_uav_offboard.uav1_fastlio_motion_test:main',
            'three_uav_waypoints = multi_uav_offboard.three_uav_waypoints:main',
            'three_uav_staggered_common_area = multi_uav_offboard.three_uav_staggered_common_area:main',
        ],
    },
)
