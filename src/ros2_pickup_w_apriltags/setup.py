from setuptools import find_packages, setup

package_name = 'ros2_pickup_w_apriltags'

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
    maintainer='ri-one',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = ros2_pickup_w_apriltags.crane_driver_node:main',
            'gui_node = ros2_pickup_w_apriltags.gui:main',
            'challenge_node = ros2_pickup_w_apriltags.challenge_node:main',
        ],
    },
)
