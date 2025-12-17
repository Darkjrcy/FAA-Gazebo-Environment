from setuptools import find_packages, setup
from glob import glob

package_name = 'airplane_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the data used in the YOLO detection system
        (f'share/{package_name}/data', glob('data/*.*')),
        (f'share/{package_name}/MIT_filtering', glob('MIT_filtering/*.*')),
        (f'share/{package_name}/data/Airplane_model', glob('data/Airplane_model/*.*')),
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
        'airplane_recognition = airplane_recognition.airplane_recognition:main',
        'MIT_spawner_and_saver = airplane_recognition.MIT_spawner_and_saver:main',
        'MIT_Trajectory_spawner = airplane_recognition.MIT_Trajectory_spawner:main',
        'camera_combination = airplane_recognition.camera_combination:main',
        'airplane_recog_fov = airplane_recognition.airplane_recog_fov:main',
        'video_generator = airplane_recognition.video_generator:main',
        'One_camera_recognition = airplane_recognition.One_camera_recognition:main',
        'variable_camera_combination = airplane_recognition.variable_camera_combination:main',
        ],
    },
)
