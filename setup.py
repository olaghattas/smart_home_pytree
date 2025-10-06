from setuptools import find_packages, setup

package_name = 'smart_home_pytree'

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
    maintainer='olagh48652',
    maintainer_email='olaghattas@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charge_robot_tree = smart_home_pytree.charge_robot_tree:charge_robot_tree_main',
            'move_to = smart_home_pytree.move_to:create_move_to_tree_main',
        ],
    },
)
