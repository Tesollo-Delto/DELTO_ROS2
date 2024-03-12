from setuptools import setup
import os
from glob import glob

package_name = 'delto_3f_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # (os.path.join('share', package_name, 'urdf'), glob('srdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # (os.path.join('share', package_name, 'config','ur3'), glob('ur3/*.yaml')),
        (os.path.join('share', package_name, 'config','ur3'), glob('ur3/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hong',
    maintainer_email='khc@tesollo.com',
    description='The ' + package_name + 'package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
