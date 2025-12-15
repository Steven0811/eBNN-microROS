from setuptools import setup, find_packages

package_name = 'mnist_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['mnist_publisher', 'mnist_publisher.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven',
    maintainer_email='steven950811@gmail.com',
    description='MNIST publisher node for sending images to micro-ROS eBNN devices',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mnist_publisher = mnist_publisher.mnist_publisher:main',
        ],
    },
    scripts=[],
)