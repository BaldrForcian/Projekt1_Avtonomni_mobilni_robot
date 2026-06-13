from setuptools import find_packages, setup

package_name = 'wt901c_RS485'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luka',
    maintainer_email='luka.kranjc1@student.um.si',
    description='WT901C-RS485 simple driver',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wt901c = wt901c_RS485.wt901c:main',
        ],
    },
)
