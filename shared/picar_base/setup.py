from setuptools import setup

package_name = 'picar_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ferin',
    maintainer_email='todo@example.com',
    description='SafetyGate and drive node for the PiCar-B (sim backend)',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'picar_base = picar_base.base_node:main',
        ],
    },
)
