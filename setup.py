from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_nav'
submodules ="autonomous_nav/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardohufg',
    maintainer_email='eduardochavezmartin10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = autonomous_nav.move:main',
            'odometry = autonomous_nav.odometry:main',
            'kalman2D = autonomous_nav.kalman2D:main',
            'odom_by_gps = autonomous_nav.odom_by_gps:main',
            'main_controller = autonomous_nav.main_controller:main',
            'nav2_controller = autonomous_nav.nav2_controller2:main',
            'laser_filter_180 = autonomous_nav.laser_filter_180:main',
        ],
    },
)
