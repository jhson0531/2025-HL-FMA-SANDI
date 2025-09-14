from setuptools import setup
import os
from glob import glob

package_name = 'nav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'control = nav_controller.control:main',
        	'utm_pure_pursuit = nav_controller.utm_pure_pursuit:main',
        	'utm_pure_pursuit_test = nav_controller.utm_pure_pursuit_test:main',
        	'utm_test_publisher = nav_controller.utm_test_publisher:main',
        	'pure_pursuit_visualizer = nav_controller.pure_pursuit_visualizer:main',
        	'simple_visualizer = nav_controller.simple_visualizer:main'
        ],
    },
)
