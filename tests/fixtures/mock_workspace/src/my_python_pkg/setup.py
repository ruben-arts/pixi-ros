from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jane Developer',
    maintainer_email='developer@example.com',
    description='A simple Python ROS 2 package',
    license='Apache-2.0',
)
