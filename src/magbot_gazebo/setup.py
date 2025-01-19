from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'magbot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description', 'urdf'), glob(os.path.join('description', 'urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'description', 'meshes'), glob(os.path.join('description', 'meshes', '*'))),
        (os.path.join('share', package_name, 'description', 'worlds'), glob(os.path.join('description', 'worlds', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jarvis',
    maintainer_email='vijayanand2k20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

        ],
    },
)
