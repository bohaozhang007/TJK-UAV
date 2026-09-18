from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['owl_nav_v22'], package_dir={'':'src'}))
