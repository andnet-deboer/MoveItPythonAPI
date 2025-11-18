"""Setup."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/planning_scene.yaml'),
        ),
        (
            os.path.join('share', package_name, 'motion_planning'),
            glob('motion_planning/*.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andnet DeBoer',
    maintainer_email='deboerandnet@gmail.com',
    description='MoveIt2 API for Python',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick = motion_planning.pick:main',
        ],
    },
)
