from setuptools import setup

package_name = 'arduino_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nader',
    maintainer_email='nwzantout@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_interface = arduino_interface.arduino_interface:main',
            'um7 = arduino_interface.um7:main',
            'keyboard_publisher = arduino_interface.keyboard_publisher:main'
        ],
    },
)
