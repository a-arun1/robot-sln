from setuptools import setup

package_name = 'sensor_pub'

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
    maintainer='aarun',
    maintainer_email='aarun@ucsd.edu',
    description='Reads 6DoF sensor values from test sensor and publishes the information in a dedicated topic',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
