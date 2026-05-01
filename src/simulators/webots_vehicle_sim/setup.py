from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'webots_vehicle_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hortejak',
    maintainer_email='hortensky.jakub@gmail.com',
    description='Webots vehicle simulation with dynamic world generation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_generator = webots_vehicle_sim.world_generator:main',
        ],
    },
)