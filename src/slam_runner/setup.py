from setuptools import find_packages, setup

package_name = 'slam_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='dalita88alitadom@hotmail.com',
    description='Python node for running FAST-LIO with bag data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_runner_node = slam_runner.slam_runner:main',
        ],
    },
)
