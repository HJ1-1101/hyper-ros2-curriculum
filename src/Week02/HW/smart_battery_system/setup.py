from setuptools import find_packages, setup

package_name = 'smart_battery_system'

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
            'battery_node = smart_battery_system.battery_node:main',
            'monitor_node = smart_battery_system.monitor_node:main',
        ],
    },
)
