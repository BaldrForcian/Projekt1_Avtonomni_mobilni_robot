from setuptools import find_packages, setup

package_name = 'keyboard_control'

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
    maintainer='feri',
    maintainer_email='feri@todo.todo',
    description='Vmesni ros2 package za povezavo ddsm115_controller knjiznice in teleop_twist_keyboard knjiznice',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "one_motor_test = keyboard_control.one_motor_test:main",
            "diff_drive_odom = keyboard_control.diff_drive_odom:main"
        ],
    },
)
