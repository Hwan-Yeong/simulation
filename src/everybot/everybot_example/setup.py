from setuptools import find_packages
from setuptools import setup

package_name = 'everybot_example'

setup(
    name=package_name,
    version='2.1.5',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # To be added
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'everybot_interactive_marker.launch.py'))),
        # ('share/' + package_name + '/launch', glob.glob(os.path.join('launch',
        #                                                 'everybot_obstacle_detection.launch.py'))),
        # ('share/' + package_name + '/rviz', glob.glob(os.path.join('rviz',
        #                                               'everybot_interactive_marker.rviz'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Ryan Shim', 'Gilbert'],
    author_email='hyjoe@everybot.net',
    maintainer='Will Son',
    maintainer_email='hyjoe@everybot.net',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of Different Everybot Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            # To be added
            # 'everybot_interactive_marker = \
            #   everybot_example.everybot_interactive_marker.main:main',
            'everybot_obstacle_detection = \
                everybot_example.everybot_obstacle_detection.main:main',
            'everybot_patrol_client = \
                everybot_example.everybot_patrol_client.main:main',
            'everybot_patrol_server = \
                everybot_example.everybot_patrol_server.main:main',
            'everybot_position_control = \
                everybot_example.everybot_position_control.main:main',
        ],
    },
)
