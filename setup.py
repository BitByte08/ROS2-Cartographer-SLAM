from setuptools import find_packages, setup
import os  # 추가
from glob import glob # 추가
package_name = 'ros2_lidar_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),# 런치 파일 설치 규칙 추가
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        # 설정 파일 설치 규칙 추가
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bitbyte08',
    maintainer_email='me@bitworkspace.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
