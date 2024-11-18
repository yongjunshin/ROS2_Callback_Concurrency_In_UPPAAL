from setuptools import setup

package_name = 'multi_thread'

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
            'external_publisher_node = multi_thread.external_publisher_node:main',
            'multi_thread_mutually_exclusive_group_node = multi_thread.multi_thread_mutually_exclusive_group_node:main',
            'multi_thread_reentrant_group_node = multi_thread.multi_thread_reentrant_group_node:main',
        ],
    },
)
