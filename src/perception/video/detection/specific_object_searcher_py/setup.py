from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'specific_object_searcher_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aboba',
    maintainer_email='121albaev@gmail.com',
    description='Specific object searcher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'specific_object_searcher_node = specific_object_searcher_py.specific_object_searcher:main',
        ],
    },
)