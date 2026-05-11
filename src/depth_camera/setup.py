from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'depth_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']), # 使用自动查找，这是最标准的写法
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 这里的路径必须使用 os.path.join 以确保准确
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsy',
    maintainer_email='lsy@todo.todo',
    description='Realsense setup package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)