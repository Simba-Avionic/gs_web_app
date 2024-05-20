from setuptools import setup

package_name = 'sim_nodes'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, "gs_ros2_utils"],# all packages in sim_nodes package + appended ones
    package_dir={"gs_ros2_utils":"../gs_ros2_utils"},
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
            'sim_mr_general_publisher = sim_nodes.sim_mr_general_publisher:main',
            'sim_radio_433_pub = sim_nodes.sim_radio_433_pub:main',
            'sim_rocket_telemetry_pub = sim_nodes.sim_rocket_telemetry_pub:main',
            'sim_rocket_status = sim_nodes.sim_rocket_status:main',
        ],
    },
)
