import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'urdf_leg'

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    version='0.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/robot_leg']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hj1',
    maintainer_email='jaewonheo1101@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
