from setuptools import find_packages, setup

package_name = 'active_hook'

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
    maintainer='Ali Ulvi Kaplan',
    maintainer_email='aliulvi4103@hotmail.com',
    description='Behaviours (action servers) for the ActiveHook ROV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spiral_search_action_server = active_hook.spiral_search:main",
        ],
    },
)
