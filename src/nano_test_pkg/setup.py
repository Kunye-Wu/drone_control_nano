from setuptools import find_packages, setup

package_name = 'nano_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nano',
    maintainer_email='nano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "open_loop_exec = nano_test_pkg.open_loop:main",
            "hover_exec = nano_test_pkg.hover:main",
            "takeoff_land_exec = nano_test_pkg.takeoff_land:main"
        ],
    },
)
