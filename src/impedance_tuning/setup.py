from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'impedance_tuning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsy',
    maintainer_email='1315827928@qq.com',
    description='Multi-joint CPG impedance tuning GUI and cycle analysis.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'impedance_tuning_gui = impedance_tuning.gui:main',
        ],
    },
)
