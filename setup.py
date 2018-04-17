from setuptools import setup

# using setuptools : http://pythonhosted.org/setuptools/
# TODO : Use the bdist_wheel setuptools extension available from the wheel project to create wheels.
# TODO : This as a third party package could be the ultimate goal...

with open('rostful/_version.py') as vf:
    exec(vf.read())

# https://hynek.me/articles/conditional-python-dependencies/
import sys
sys_requires = []
if sys.version_info[0:3] <= (2, 7, 9) or (3, 0) < sys.version_info[0:2] <= (3, 4):
    sys_requires += ['backports.ssl-match-hostname']


setup(
    name='rostful',
    version=__version__,
    description='REST API for ROS',
    url='http://github.com/asmodehn/rostful',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
    packages=[
        'rostful',
        'rostful.api_0_1',
        'rostful.api_0_2',
        'rostful.frontend',
        #'rostful.tests',  # not embedding tests in package for now...
    ],
    package_dir={
    },
    entry_points={
        'console_scripts': [
            'rostful = rostful.__main__:cli'
        ]
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    package_data={  # TODO : might be better to do this in MANIFEST
        'rostful': [
            'static/favicon.ico',
            'static/js/moment/*',
            'static/js/jquery/*',
            'static/js/jquery-mobile/jquery.*',
            'static/js/jquery-mobile/images/ajax-loader.gif',
            'static/js/jquery-mobile/images/icons-png/*',
            'static/js/jquery-mobile/images/icons-svg/*',
            'templates/*.html',
            'templates/security/*.html',
            'templates/security/email/*',
        ],
    },
    install_requires=sys_requires + [
        'futures>=3.0.2',
        'Flask>=0.10.1',
        'Flask-Cors>=3.0.2',
        #'Flask-Script',
        'Flask-Restful>=0.3.6',
        'Flask-reverse-proxy',
        #'Rester',
        'click>=6.4.0',
        'webargs>=1.3.4',
        'pyros>=0.3.0',
        'pyros_setup>=0.1.5',  # pyros should provide this...
        'pyros_config>=0.1.4',  # pyros should provide this...
        'tornado>=4.2.1, <5.0',  # untested with tornado > 5.0
        'simplejson',
        'tblib>=1.2',
    ],
    test_suite="nose.collector",
    tests_require=[
        'nose>=1.3.7'
    ],
    zip_safe=False,  # TODO testing...
)

