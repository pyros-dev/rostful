## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'rostful',
    ],
    install_requires=[
        'rostful_node',
        'Flask',
        # TODO : list dependencies here to download them and avoid using flask_ext_catkin ??
    ],
    # not usable with catkin distutils version.
    # Should work with setuptools.setup. Will include files from Manifest.in
    # include_package_data=True,
    package_data={
        'rostful': [
            'static/js/jquery/*',
            'static/js/jquery-mobile/*',
            'static/js/jquery-mobile/images/*',
            'static/js/jquery-mobile/images/icons-png/*',
            'static/js/jquery-mobile/images/icons-svg/*',
            'templates/*',
            'templates/security/*',
            'templates/security/email/*',
        ],
    },
)

setup(**setup_args)
