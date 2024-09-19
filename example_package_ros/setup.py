from setuptools import setup
import os
from glob import glob

package_name = 'example_package_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Register package with ament index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include test files
        (os.path.join('share', package_name, 'test'), glob('test/*.py')),
    ],
    install_requires=['setuptools', 'example_package'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Example Name',
    maintainer_email='example@example.com',
    description='ROS-dependent example package',
    # license='TODO',
    entry_points={
        'console_scripts': [
            'example_node = example_package_ros.example_node:main',
            'advanced_example_node = example_package_ros.advanced_example_node:main',
        ],
    },
)
