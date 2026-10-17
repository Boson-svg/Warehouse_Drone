import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_bringup'


def package_files(directory, install_root):
    data_files = []
    for path, _, filenames in os.walk(directory):
        if not filenames:
            continue

        install_path = os.path.join(
            "share", package_name, install_root, os.path.relpath(path, directory)
        )
        files = [os.path.join(path, name) for name in filenames]
        data_files.append((install_path, files))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ] + package_files("sim", "sim") + package_files("scripts", "scripts"),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
