from setuptools import find_packages, setup

package_name = 'move_to_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chukotskiy_shaman',
    maintainer_email='s.mendrul@g.nsu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_goal_node = move_to_goal.move_to_goal:main',
        ],
    },
)
