from setuptools import find_packages, setup

package_name = 'oakd_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/oakd_ros2_launch.py']),
        ('share/' + package_name + '/launch', ['launch/multicams_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st',
    maintainer_email='119010261@link.cuhk.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_ros2_publisher = oakd_ros2_driver.oakd_ros2_publisher:main',
            'multicams_publisher = oakd_ros2_driver.multicams_publisher:main',
        ],
    },
)
