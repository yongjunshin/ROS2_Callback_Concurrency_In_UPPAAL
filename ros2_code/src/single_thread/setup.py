from setuptools import setup

package_name = 'single_thread'

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
    maintainer='yjshin',
    maintainer_email='yjshin@etri.re.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'external_publisher_node = single_thread.external_publisher_node:main',
            'single_thread_node = single_thread.single_thread_node:main',
        ],
    },
)
