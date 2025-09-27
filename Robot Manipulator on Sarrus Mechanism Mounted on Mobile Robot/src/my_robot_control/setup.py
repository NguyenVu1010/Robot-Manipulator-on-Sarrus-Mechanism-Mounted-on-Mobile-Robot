## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robot_lib'], # Tên của thư mục thư viện của bạn
    package_dir={'': 'src'}   # Nơi để tìm các package (từ thư mục src)
)

setup(**setup_args)