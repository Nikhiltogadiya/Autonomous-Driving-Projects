from setuptools import setup

package_name = 'fusion_grid'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Ohl',
    maintainer_email='s.ohl@ostfalia.de',
    description='grid fusion',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_driver = fusion_grid.fusion_driver:main',
            'grid = fusion_grid.lidar_grid:main',
        ],
    },
)
