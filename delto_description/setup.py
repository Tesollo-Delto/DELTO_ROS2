from setuptools import setup
import os
from glob import glob

package_name = 'delto_description'

current_dir = os.path.dirname(os.path.realpath(__file__))

config_ur3_rel_dir = os.path.relpath(os.path.join(current_dir, 'config', 'ur3'))
mesh_DG2F_rel_dir = os.path.relpath(os.path.join(current_dir, 'meshes', 'DG2F'))
mesh_DG3F_rel_dir =  os.path.relpath(os.path.join(current_dir, 'meshes', 'DG3F'))
mesh_DG5F_rel_dir =  os.path.relpath(os.path.join(current_dir, 'meshes', 'DG5F'))

current_dir = os.path.dirname(os.path.realpath(__file__))
ur3_dir = os.path.join(current_dir, 'ur3')

ur3_rel_dir = os.path.relpath(ur3_dir)
config_ur3_rel_dir = os.path.relpath(os.path.join(current_dir, 'config', 'ur3'))


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
        (os.path.join('share', package_name, 'meshes','DG2F'), glob(os.path.join(mesh_DG2F_rel_dir, '*'))),
        (os.path.join('share', package_name, 'meshes','DG3F'), glob(os.path.join(mesh_DG3F_rel_dir, '*'))),
        (os.path.join('share', package_name, 'meshes','DG5F'), glob(os.path.join(mesh_DG5F_rel_dir, '*'))),

        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config','ur3'), glob('config/ur3/*')),
        (os.path.join('share', package_name, 'config','ur3'), glob(os.path.join(ur3_rel_dir, '*.yaml'))),
        (os.path.join('share', package_name, 'config','ur3'), glob(os.path.join(config_ur3_rel_dir, '*'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hong',
    maintainer_email='khc@tesollo.com',
    description='The ' + package_name + 'package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        [
        ],
    },
)
