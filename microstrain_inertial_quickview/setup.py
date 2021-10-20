import os
import glob
from setuptools import setup, find_packages

package_name = 'microstrain_inertial_quickview'

common_dir = 'microstrain_inertial_quickview_common'
common_src_dir = os.path.join(common_dir, 'src')
common_resource_dir = os.path.join(common_dir, 'resource')

dest_share_dir = os.path.join('share', package_name)
dest_common_share_dir = os.path.join(dest_share_dir, common_dir)
dest_common_resource_dir = os.path.join(dest_common_share_dir, 'resource')

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': common_src_dir},
    packages=find_packages(common_src_dir),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (dest_share_dir, ['package.xml']),
        (dest_share_dir, glob.glob('launch/*')),
        (dest_common_share_dir, [os.path.join(common_dir, 'plugin.xml')]),
        (dest_common_resource_dir, glob.glob(os.path.join(common_resource_dir, '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rob Fisher',
    maintainer_email='rob.fisher@parker.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='microstrain_inertial_quickview provides a simple interface to view Microstrain Inertial devices',
    license='MIT',
    tests_require=['pytest'],
)
