from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pioneer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # src subdirectories
        (os.path.join('share', package_name, 'src', 'meshes', 'p3at_meshes'),
            glob('src/meshes/p3at_meshes/*')),
        (os.path.join('share', package_name, 'src', 'meshes'),
            [f for f in glob('src/meshes/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'src', 'robots'),
            glob('src/robots/*')),
        (os.path.join('share', package_name, 'src', 'worlds'),
            glob('src/worlds/*')),


        # launch directory
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),

        # rviz directory
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),

        # config
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share', package_name), ['pioneer/waypoints.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeremy',
    maintainer_email='jeremynbutson@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'velocity_publisher = pioneer.velocity_publisher:main',
        ],
    },
)
