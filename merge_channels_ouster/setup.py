from setuptools import setup

package_name = 'merge_channels_ouster'
setup(
    name=package_name,
    version='1.0.0',
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    packages=[package_name],
    maintainer='Edison Velasco',
    maintainer_email="epvelasco1912@gmail.com",
    description='Merge LiDAR channels package',
    license='BSD',
    entry_points={
        'console_scripts': [
            'merge_channels_ouster = merge_channels_ouster.merge_channels_ouster:main',
            'save_merge_channels_ouster = merge_channels_ouster.save_merge_channels_ouster:main',
        ],
    },
)
