from setuptools import setup

package_name = 'rm_camera_demo'

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
    maintainer='a',
    maintainer_email='a@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sub_image_node = rm_camera_demo.camera_visual_demo:main',
        'Center_Coordinate_node = rm_camera_demo.show_center_coordinate:main',
        'camera_0_node = rm_camera_demo.camera_0:main',
        'camera_1_node = rm_camera_demo.camera_1:main',
        'open_realsense_node = rm_camera_demo.open_realsense_camera:main',
        'realsense_camera_0_node = rm_camera_demo.realsense_camera_0:main',
        'realsense_camera_1_node = rm_camera_demo.realsense_camera_1:main',
        'realsense_camera_2_node = rm_camera_demo.realsense_camera_2:main',
        ],
    },
)
