from setuptools import setup

package_name = 'nav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        	'utm_path_launcher = nav_controller.utm_path_launcher:main',
        	'utm_path_generator = nav_controller.utm_path_generator:main',
        	'utm_path_controller = nav_controller.utm_path_controller:main',
        	'utm_path_with_gps = nav_controller.utm_path_with_gps:main'
        ],
    },
)
