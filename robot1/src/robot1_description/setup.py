from setuptools import setup
import os
from glob import glob

package_name = 'robot1_description'

# 获取launch和config文件夹下的所有文件（自动识别，不用手动加）
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 自动添加launch文件夹下的所有.launch.py文件
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # 自动添加config文件夹下的所有.yaml文件
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    # 自动添加urdf文件夹下的所有文件（如果有）
    (os.path.join('share', package_name, 'urdf'), glob('urdf/**/*', recursive=True)),
    # 自动添加meshes文件夹下的所有模型文件（如果有）
    (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mshist',  # 改成你的用户名（比如mshist）
    maintainer_email='mshist@example.com',  # 随便填，不影响运行
    description='Robot1 URDF and MoveIt Servo configuration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 注册键盘控制节点，这是能运行脚本的核心
            'keyboard_absolute_pose = robot1_description.keyboard_absolute_pose:main',
        ],
    },
)

