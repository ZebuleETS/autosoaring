from setuptools import setup

package_name = 'autosoaring_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autosoaring_launch.py']),
        ('share/' + package_name + '/config', [
            'config/thermal_config.yaml', 
            'config/area2.plan',
            'config/area3.plan'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autosoaring Team',
    maintainer_email='user@example.com',
    description='Autonomous soaring package with thermal detection, generation, mapping and battery management',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_manager_node = autosoaring_pkg.battery_manager_node:main',
            'thermal_detection_node = autosoaring_pkg.thermal_detection_node:main',
            'thermal_generator_node = autosoaring_pkg.thermal_generator_node:main',
            'thermal_mapping_node = autosoaring_pkg.thermal_mapping_node:main',
        ],
    },
)
