from setuptools import setup
import os
from glob import glob
from setuptools import find_packages


package_name = 'lab3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafal',
    maintainer_email='you@example.com',
    description='description',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = lab3.joint_state_publisher:main',
            'nonkdl_dkin = lab3.nonkdl_dkin:main'
        ],
    },
)
