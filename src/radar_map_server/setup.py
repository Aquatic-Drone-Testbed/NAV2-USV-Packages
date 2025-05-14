from setuptools import setup
import os
from glob import glob

package_name = 'radar_map_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
    install_requires=['setuptools','numpy'],
    entry_points={
        'console_scripts': [
            'heatmap_publisher = radar_map_server.heatmap_publisher:main'
        ],
    },
)
