from setuptools import find_packages, setup
import os
package_name = 'vcon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'vcon'), ['VCON.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hortejak',
    maintainer_email='hortensky.jakub@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vcon_publisher = vcon.vcon_publisher:main'
        ],
    },
)
