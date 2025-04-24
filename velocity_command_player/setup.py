from setuptools import find_packages, setup

package_name = 'velocity_command_player'

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
    maintainer='ethan',
    maintainer_email='eholand@andrew.cmu.edu',
    description='Play back CSV velocity commands to a forward_command_controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csv_velocity_player = velocity_command_player.csv_velocity_player:main'
        ],
    },
)
