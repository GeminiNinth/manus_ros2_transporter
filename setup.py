from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manus_ros2_transporter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GeminiNinth',
    maintainer_email='gemini.ninth@gmail.com',
    description='ZeroMQ-based transporter for Manus glove data between PCs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zmq_bridge = manus_ros2_transporter.zmq_bridge:main',
            'zmq_receiver = manus_ros2_transporter.zmq_receiver:main',
        ],
    },
)

