from setuptools import find_packages, setup

package_name = 'perception_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/safety_stop.yaml',
            'config/self_filter.yaml',
            'config/sidewalk_polygon.yaml',
            'config/sidewalk_polygon_w5.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='LiDAR safety stop for Go2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'collision_oracle_node = perception_avoidance.collision_oracle_node:main',
            'lidar_obstacle_node = perception_avoidance.lidar_obstacle_node:main',
            'safety_stop_node = perception_avoidance.safety_stop_node:main',
            'degrade_pointcloud_node = perception_avoidance.degrade_pointcloud_node:main',
            'sidewalk_polygon_node = perception_avoidance.sidewalk_polygon_node:main',
        ],
    },
)
