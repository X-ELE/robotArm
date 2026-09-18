from setuptools import setup

package_name = 'robot_arm_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if False else []),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotica Educativa',
    maintainer_email='docencia@robotica.edu',
    description='Nodo puente ROS 2 a MKS Gen v1.4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = robot_arm_bridge.serial_bridge_node:main',
        ],
    },
)
