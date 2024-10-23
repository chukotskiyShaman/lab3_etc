from setuptools import find_packages, setup

package_name = 'ex03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtle_time_travel.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chukotskiy_shaman',
    maintainer_email='s.mendrul2010@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle1_tf2_broadcaster = ex03.turtle1_tf2_broadcaster:main',
            'turtle2_tf2_broadcaster = ex03.turtle2_tf2_broadcaster:main',
            'turtle2_tf2_listener = ex03.turtle2_tf2_listener:main',
        ],
    },
)
