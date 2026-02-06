from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parking_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Correctly install all files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'web'), glob(os.path.join('web', '*'))),
        
        # Correctly install the model and its contents
        (os.path.join('share', package_name, 'models/indoor_garage'), glob(os.path.join('models/indoor_garage', '*'))),
        (os.path.join('share', package_name, 'models/indoor_garage/meshes'), glob(os.path.join('models/indoor_garage/meshes', '*'))),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='madhukar',
    maintainer_email='madhukar@todo.todo',
    description='A web dashboard and management system for autonomous parking with Autoware.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'integrated_dashboard_node = parking_system.integrated_dashboard_node:main',
        ],
    },
)
