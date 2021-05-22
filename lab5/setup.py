from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'lab5'

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
        'nonkdl_dkin = lab5.nonkdl_dkin:main',
            'kdl_dkin = lab5.kdl_dkin:main',
            'jint = lab5.jint:main',
            'service_oint = lab5.service_oint:main',
            'client_oint = lab5.client_oint:main',
            'client_jint = lab5.client_jint:main',
            'ikin = lab5.ikin:main'
        ],
    },
)
