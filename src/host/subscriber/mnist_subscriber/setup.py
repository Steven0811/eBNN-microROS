from setuptools import setup, find_packages

package_name = 'mnist_subscriber'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['mnist_subscriber', 'mnist_subscriber.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven',
    maintainer_email='steven950811@gmail.com',
    description='MNIST subscriber node for fetching results from micro-ROS eBNN devices',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mnist_subscriber = mnist_subscriber.mnist_subscriber:main',
        ],
    },
    scripts=[],
)