from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['movo_demo'],
    package_dir={'': 'scripts'}
)
setup(**d)