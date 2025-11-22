from setuptools import find_packages, setup

package_name = 'rm_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/visualization.launch.py',
             'launch/test_visualization.launch.py']),
        ('share/' + package_name + '/config', 
            ['config/visualization_params.yaml',
             'config/DartV1.0.ui']),
    ],
    install_requires=['setuptools', 'rm_interfaces'],
    zip_safe=True,
    maintainer='lenovo',
    maintainer_email='2126948842@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dart_ui = rm_visualization.ui_dart_main:main"
        ],
    },
    package_data={
        'rm_visualization': ['../config/*.ui'],
    },
    include_package_data=True,
)
