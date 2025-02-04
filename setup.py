import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'biorobotics_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models'), glob('models/*/*')),
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
        'spawn_bot = biorobotics_tutorial.spawn_bot:main',
        'two_link_controller = biorobotics_tutorial.two_link_controller:main'
        'two_link_pid_controller = biorobotics_tutorial.two_link_pid_controller:main'
        ],
    },
)
