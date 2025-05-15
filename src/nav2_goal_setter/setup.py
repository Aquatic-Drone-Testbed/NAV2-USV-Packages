from setuptools import find_packages, setup

package_name = 'nav2_goal_setter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seamate-nav-docker',
    maintainer_email='seamate-nav-docker@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_goal_setter = nav2_goal_setter.goal_publisher:main'
        ],
    },
)
