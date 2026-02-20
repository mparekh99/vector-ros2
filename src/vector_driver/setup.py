from setuptools import find_packages, setup

package_name = 'vector_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
	('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vector_nav2_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihir',
    maintainer_email='mihir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'vector_node = vector_driver.vector_node:main',
		# 'drive_square = vector_driver.drive_square:main',
		# 'teleop = vector_driver.teleop:main',
        'marker_localization_node = vector_driver.marker_localization_node:main'
	],
    },
)
