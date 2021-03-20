from setuptools import setup

package_name = 'lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kacper',
    maintainer_email='kacperm102@gmail.com',
    description='Lab1 - zadanie',
    license='licencja',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_teleop = lab1.my_teleop:main'
        ],
    },
)
