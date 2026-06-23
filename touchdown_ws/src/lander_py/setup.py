from setuptools import find_packages, setup

package_name = 'lander_py'

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
    maintainer='antonino1702',
    maintainer_email='antoniobiondo1702@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thrusters_node = lander_py.thrusters:main',
            'data_logger_node = lander_py.data_logger:main',
            'thruster_logger_node = lander_py.thruster_logger:main',
            'generate_body = lander_py.generate_body:generate_simulation_files',
            'generate_scenario = lander_py.generate_scenario:generate_kinematics',
            'generate_legs = lander_py.generate_legs:main',
        ],
    },
)
