from setuptools import setup

package_name = 'robot_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name+".action", package_name+".verifier"],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Ohl',
    maintainer_email='s.ohl@ostfalia.de',
    description='robot test framework',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testconfiguration = robot_test.testconfigurationNode:main'
        ],
    },
)
