from setuptools import find_packages, setup

package_name = 'rover_controls'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
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
    maintainer='Šimon Prokop',
    maintainer_email='221488@vut.cz',
    description='Control and simulated base nodes for the Rover project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'safety_watchdog_node = '
                'rover_controls.safety_watchdog_node:main'
            ),
            (
                'base_simulator_node = '
                'rover_controls.base_simulator_node:main'
            ),
        ],
    },
)
