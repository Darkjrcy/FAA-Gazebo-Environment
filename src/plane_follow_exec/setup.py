from setuptools import find_packages, setup

package_name = 'plane_follow_exec'

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
            'mit_encounter_follower = plane_follow_exec.mit_encounter_follower:main',
            'daa_simulation = plane_follow_exec.daa_simulation:main',
        ],
    },
)
