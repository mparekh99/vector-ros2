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
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
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
            'drive_square = vector_driver.drive_square:main',
            # 'test_logger_node = vector_driver.test_logger_node:main',
            # 'teleop = vector_driver.teleop:main',
            'marker_localization_node = vector_driver.marker_localization_node:main',
            'ekf_node = vector_driver.ekf_node:main',
            'ekf_test_driver = vector_driver.ekf_test_driver:main',
            'drive_forward = vector_driver.drive_forward:main',
            'multi_topic_logger = vector_driver.multi_topic_logger:main',
	        # 'workspace_visualizer_node = vector_driver.workspace_visualizer_node:main'
	],
    },
)
