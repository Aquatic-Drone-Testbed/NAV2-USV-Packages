import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'usv_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.*'))),
        #Include model and simulation files
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*'))),
        #Include params
        (os.path.join('share', package_name, 'params'), 
            glob(os.path.join('params', '*'))),
        #Include config
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seamate-docker',
    maintainer_email='seamate-docker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_odom = usv_nav2.custom_odom_pub:main',
            'custom_odom_imu = usv_nav2.custom_odom_pub_imu:main',
        ],
    },
)
