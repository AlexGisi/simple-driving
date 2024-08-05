from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["simple_driving_control"],  # Python packages to install
    package_dir={"": "src"},  # Directory where Python packages are located
    scripts=[
        "scripts/control_node.py",
        "scripts/log_node.py",
    ],  # Scripts to be installed
)

setup(**setup_args)
