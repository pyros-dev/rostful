import setuptools
import subprocess

# using setuptools : http://pythonhosted.org/setuptools/

with open('rostful/_version.py') as vf:
    exec(vf.read())


# Best Flow :
# Clean previous build & dist
# $ gitchangelog >CHANGELOG.rst
# change version in code and changelog (AND in package .xml for ROS !)
# $ python setup.py prepare_release
# WAIT FOR TRAVIS CHECKS
# $ python setup.py publish
# => TODO : try to do a simpler "release" command

#
# And then for ROS, directly (tag is already in git repo):
# bloom-release --rosdistro kinetic --track kinetic pyros_interfaces_ros


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PrepareReleaseCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "prepare a release of pyros"
    user_options = []

    def initialize_options(self):
        """init options"""
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""

        # TODO :
        # $ gitchangelog >CHANGELOG.rst
        # change version in code and changelog
        subprocess.check_call("git commit CHANGELOG.rst rostful/_version.py -m 'v{0}'".format(__version__), shell=True)
        subprocess.check_call("git push", shell=True)

        print("You should verify travis checks, and you can publish this release with :")
        print("  python setup.py publish")
        sys.exit()


# Clean way to add a custom "python setup.py <command>"
# Ref setup.py command extension : https://blog.niteoweb.com/setuptools-run-custom-code-in-setup-py/
class PublishCommand(setuptools.Command):
    """Command to release this package to Pypi"""
    description = "releases pyros to Pypi"
    user_options = []

    def initialize_options(self):
        """init options"""
        # TODO : register option
        pass

    def finalize_options(self):
        """finalize options"""
        pass

    def run(self):
        """runner"""
        # TODO : clean build/ and dist/ before building...
        subprocess.check_call("python setup.py sdist", shell=True)
        subprocess.check_call("python setup.py bdist_wheel", shell=True)
        # OLD way:
        # os.system("python setup.py sdist bdist_wheel upload")
        # NEW way:
        # Ref: https://packaging.python.org/distributing/
        subprocess.check_call("twine upload dist/*", shell=True)

        subprocess.check_call("git tag -a {0} -m 'version {0}'".format(__version__), shell=True)
        subprocess.check_call("git push --tags", shell=True)
        sys.exit()



# https://hynek.me/articles/conditional-python-dependencies/
import sys
sys_requires = []
if sys.version_info[0:3] <= (2, 7, 9) or (3, 0) < sys.version_info[0:2] <= (3, 4):
    sys_requires += ['backports.ssl-match-hostname']


setuptools.setup(
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
        #'futures>=3.0.2',
        'Flask>=0.10.1',
        'Flask-Cors>=3.0.2',
        'Flask-Restful>=0.3.4',
        'Flask-reverse-proxy',
        'click>=6.2.0',
        'webargs>=1.3.4',
        'pyros>=0.4.3',
        #'pyros_setup>=0.1.5',  # pyros should provide this...
        #'pyros_config>=0.1.4',  # pyros should provide this...
        'tornado>=4.2.1, <5.0',  # untested with tornado > 5.0
        'simplejson',
        'tblib>=1.2',
    ],
    test_suite="nose.collector",
    tests_require=[
        'nose>=1.3.7'
    ],
    cmdclass={
        'prepare_release': PrepareReleaseCommand,
        'publish': PublishCommand,
    },
    zip_safe=False,  # TODO testing...
)

