from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_primitives'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config/arms', glob('config/arms/*.yaml')),
    ],
    package_data={package_name: ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johnalbert',
    maintainer_email='johnnyas@u.northwestern.edu',
    description='Arm-agnostic motion, gripper and scene primitives for the Franka FER and UR7e',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'primitives_node = arm_primitives.primitives_node:main',
            'demo = arm_primitives.demo:main',
        ],
    },
)
