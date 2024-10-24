from setuptools import find_packages, setup

package_name = 'apriltag_control'

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
    maintainer='shouzaki',
    maintainer_email='sho.uzaki@icloud.com',
    description='Determine the robot`s position and orientation using Apriltag',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = apriltag_control.control_node:main',
            'sub_node = apriltag_control.sub_node:main',
            'action_server = apriltag_control.action_server:main',
            'action_client = apriltag_control.action_client:main',
        ],
    },
)
