from setuptools import setup

package_name = 'solution_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Reflect',
    maintainer_email='youremail@example.com',
    description='ROS 2 Capstone Project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'm_h_tc = solution_package.m_h_tc:main',
        ],
    },
)
