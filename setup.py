from glob import glob
from setuptools import setup

package_name = 'osp_explorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 6',
    maintainer_email='team6@example.com',
    description='ArUco marker mapping (camera bearing + LiDAR range)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'marker_mapper = osp_explorer.marker_mapper:main',
        ],
    },
)
