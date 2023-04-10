from setuptools import setup

package_name = 'ackertocan'

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
    maintainer='Yacht Chen',
    maintainer_email='ych363@aucklanduni.ac.nz',
    description='The package include a node that convert Ackermann steering msg to CAN msg',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker1 = ackertocan.Ackerman:main',
                'listener1 = ackertocan.conversion:main',
                'listener2 = ackertocan.CAN:main',
        ],
    },
)
