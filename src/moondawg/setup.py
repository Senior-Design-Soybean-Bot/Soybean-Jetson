import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moondawg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew',
    maintainer_email='andrew.barnes@siu.edu',
    description='Package running on the Raspberry Pi.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_translator = moondawg.xbox_translator:main',
            'i2c_bridge = moondawg.i2c_bridge:main',
            'gps_publisher = moondawg.gps_publisher:main',
            'image_capture = moondawg.image_capture:main',
        ],
    },
)
