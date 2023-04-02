from setuptools import setup

package_name = 'py_pubsub'

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
    maintainer='jtsizoo',
    maintainer_email='sizoo.justin@gmail.com',
    description='CADD-E UI ROS Messaging',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'tee_publisher = py_pubsub.tee_publisher:main',
             'go_publisher = py_pubsub.go_publisher:main',
             'stop_publisher = py_pubsub.stop_publisher:main',
             'bool_subscriber = py_pubsub.bool_subscriber:main',
             'string_subscriber = py_pubsub.string_subscriber:main'
        ],
    },
)
