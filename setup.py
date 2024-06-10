import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ubisense_ros2_port'

# all the data files, individual ones
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ]



# custom function to copy folders to the install directory
def package_files(data_files, directory_list):
    # check if the directory list is empty
    if not directory_list:
        return data_files
    
    # else, add the files in the directory to the data_files
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=package_files(data_files, []),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rejin',
    maintainer_email='rejin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ubisense_ros2_port.publisher_node:main',
        ],
    },
)
