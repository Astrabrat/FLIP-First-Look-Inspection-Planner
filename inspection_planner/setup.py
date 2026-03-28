from setuptools import setup

package_name = 'inspection_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py', 'launch/voxel_grid_simple.launch.py']),
        ('share/' + package_name + '/config', ['config/rosparams_exp.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vignesh Kottayam Viswanathan',
    maintainer_email='vigkotvis@gmail.com',
    description='inspection_planner (ROS 2 port)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'planner_node = inspection_planner.planner_node:main',
        ],
    },
)
