from setuptools import find_packages, setup

package_name = 'vegabot_scripts'

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
    maintainer='utk',
    maintainer_email='dhruti1357@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_lidar = vegabot_scripts.read_lidar:main',
            'read_imu = vegabot_scripts.read_imu:main',
            'read_camera = vegabot_scripts.read_camera:main',
            'detect_marker = vegabot_scripts.detect_marker:main',
            'maze_solver = vegabot_scripts.maze_solver:main',
            'obstacle_avoidance = vegabot_scripts.obstacle_avoidance:main',
            'auto_docking_undocking = vegabot_scripts.auto_docking_undocking:main',
            'docking_with_patrolling = vegabot_scripts.docking_with_patrolling:main',
            'auto_docking_with_battery = vegabot_scripts.auto_docking_with_battery:main',
        ],
    },
)
