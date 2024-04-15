from setuptools import setup

package_name = 'sensor_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f"{package_name}/third_party"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarun',
    maintainer_email='aarun@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service = sensor_pub.sensor_server:main',
        'client1 = sensor_pub.sensor_cli:main1',
        'client2 = sensor_pub.sensor_cli:main2',
        ],
    },
)
