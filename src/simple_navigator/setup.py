from setuptools import setup

package_name = 'simple_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_navigator_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR NAME',
    maintainer_email='you@example.com',
    description='Simple auto-navigation node without AprilTags',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_navigator = simple_navigator.simple_navigator:main'
        ],
    },
)
