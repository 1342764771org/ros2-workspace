from setuptools import find_packages, setup
from glob import glob

package_name = 'system_status_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zack',
    maintainer_email='1342764771@qq.com',
    description='a tool to monitor the present status of the pc',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_status_publisher=system_status_monitoring.system_status_publisher:main',
            'system_status_subscriber=system_status_monitoring.system_status_subscriber:main',
        ],
    },
)
