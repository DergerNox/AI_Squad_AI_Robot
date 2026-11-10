from glob import glob

from setuptools import setup

package_name = 'picar_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ferin',
    maintainer_email='todo@example.com',
    description='The real PiCar-B: distance sensor node and car.launch.py',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ultrasonic = picar_car.ultrasonic_node:main',
        ],
    },
)
