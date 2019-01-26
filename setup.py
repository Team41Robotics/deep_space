## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

def read(fname):
    return open(fname).read()

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['lidar_processor','vision_processor','hatch_tf_broadcaster','map_tf_broadcaster'],
    package_dir={'': 'src'},
    platforms=['ROS/t_41_2019']
)

setup(**setup_args)
