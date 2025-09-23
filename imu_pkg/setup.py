from setuptools import setup
package_name = 'imu_pkg'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', ['launch/imu_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='IAHRS IMU reader',
    license='MIT',
    entry_points={'console_scripts': [
        'imu_reader_node = imu_pkg.imu_reader_node:main',
        'imu_processed_node = imu_pkg.imu_processed_node:main',
        'imu_raw_node = imu_pkg.imu_raw_node:main',
        ]},
)

