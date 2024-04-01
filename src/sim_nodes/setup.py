from setuptools import find_packages, setup

package_name = 'sim_nodes'

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
    maintainer='ziolko',
    maintainer_email='ziolko@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_valve_sensors_pub = sim_nodes.sim_valve_sensors_pub:main',
            'sim_valve_servos_pub = sim_nodes.sim_valve_servos_pub:main',
            'sim_load_cells = sim_nodes.sim_load_cells:main',
        ],
    },
)
