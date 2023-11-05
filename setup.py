from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
a = generate_distutils_setup(
    packages=['thyroid_ultrasound_robot_control_support'],
    package_dir={'': 'src'}
    )
setup(**a)
