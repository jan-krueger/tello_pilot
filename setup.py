from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['src/tello_pilot_node.py', 'src/ap_setup_node.py'],
    packages=['tello_pilot'],
    package_dir={'': 'src'}
)

setup(**d)
