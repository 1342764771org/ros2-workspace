from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_recognition_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zack',
    maintainer_email='1342764771@qq.com',
    description='Face recognition functionality with client and server nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detection_server = face_recognition_pkg.face_detection_server:main',
            'face_detection_client = face_recognition_pkg.face_detection_client:main',
        ],
    },
)
