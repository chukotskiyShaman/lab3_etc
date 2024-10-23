from setuptools import setup

package_name = 'ex02'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlesim_with_carrot.launch.py']),
        ('share/' + package_name + '/config', ['config/carrot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_tf2_broadcaster = ex02.turtle1_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = ex02.turtle2_tf2_broadcaster:main',
            'carrot_tf2_broadcaster = ex02.carrot_tf2_broadcaster:main',
            'turtle2_tf2_listener = ex02.turtle2_tf2_listener:main',
        ],
    },
)
