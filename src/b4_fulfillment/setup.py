from setuptools import find_packages, setup

package_name = 'b4_fulfillment'

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
    maintainer='yeeun',
    maintainer_email='ye971120@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = b4_fulfillment.gui:main',
            'webcam = b4_fulfillment.webcam:main',
            'robot = b4_fulfillment.robot:main',
            'conveyor = b4_fulfillment.conveyor:main',
            'manipulation = b4_fulfillment.manipulation:main',
            'robot_eyes = b4_fulfillment.robot_eyes:main',
            'debug = b4_fulfillment.debug:main'
        ],
    },
)
