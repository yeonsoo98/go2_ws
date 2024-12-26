from setuptools import setup

package_name = 'local_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/integrated_planner_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeonsoo',
    maintainer_email='youremail@example.com',
    description='Local path planner with integrated costmap',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap = local_path_planner.costmap:main',  
            'path_planner = local_path_planner.path_planner:main',  
        ],
    },
)
