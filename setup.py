from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mycobot_jointstate_controller'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pymycobot',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='inoma',
    maintainer_email='inoma@users.noreply.github.com',
    description='ROS 2 JointState subscriber controller for myCobot using pymycobot.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'jointstate_controller_node = mycobot_jointstate_controller.jointstate_controller_node:main',
        ],
    },
)
