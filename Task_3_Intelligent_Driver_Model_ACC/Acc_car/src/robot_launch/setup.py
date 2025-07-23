"""webots_ros2 package setup file."""

from setuptools import setup

package_name = 'robot_launch'
data_files = []
#data_files.append(('share/ament_index/resource_index/packages',
#                   ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',
                   ['launch/cloud_launch.py', 'launch/local_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/tesla_world.wbt',
    'worlds/.tesla_world.wbproj',
]))
data_files.append(('share/' + package_name + '/worlds/tesla_world_net', [
    'worlds/tesla_world_net/sumo.add.xml',
    'worlds/tesla_world_net/sumo.edg.xml',
    'worlds/tesla_world_net/sumo.net.xml',
    'worlds/tesla_world_net/sumo.netccfg',
    'worlds/tesla_world_net/sumo.nod.xml',
    'worlds/tesla_world_net/sumo.rou.xml',
    'worlds/tesla_world_net/sumo.sumocfg',
    'worlds/tesla_world_net/viewsettings.xml',
]))
data_files.append((
    'share/' + package_name + '/resource',
    [
        'resource/robot_webots.urdf',
    ]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Sebastian Ohl',
    author_email='s.ohl@ostfalia.de',
    maintainer='',
    maintainer_email='',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Tesla ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
