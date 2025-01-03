from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'magbot_peripheral_interfacing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'lib'), glob(os.path.join('lib', '*'))),
    ],
    install_requires=[
        'setuptools',
        'RPi.GPIO',
        'Pillow',
        'spidev',
        'numpy',
        'adafruit-circuitpython-bno055',
        'board'
    ],
    zip_safe=True,
    maintainer='jarvis',
    maintainer_email='vijayanand2k20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "magbot_lcd_interfacing = magbot_peripheral_interfacing.magbot_lcd_interfacing:main",
            # "imu = magbot_peripheral_interfacing.IMU:main",
        ],
    },
)
