from setuptools import find_packages, setup

package_name = 'save_information'

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
    maintainer='adcl',
    maintainer_email='jorgestu20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'save_airplane_information = save_information.save_airplane_Information:main',
        'save_performance_metrics = save_information.save_performance_metrics:main',
        'save_detection_information = save_information.save_detection_data:main',
        'stop_simulation = save_information.stop_simulation:main',
        'gazebo_waiter = save_information.gazebo_waiter:main',
        'delete_entity = save_information.delete_entity:main',
        ],
    },
)
