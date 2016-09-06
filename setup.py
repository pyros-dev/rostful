from setuptools import setup

with open('rostful/_version.py') as vf:
    exec(vf.read())

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
        'rostful.tests',
        # This can create potential conflicts in install space,
        # If another package install the same pthon package dependency.
        # TODO : The proper solution is to create ThirdPartyRelease for these packages
        'rester',
        'testfixtures',
        'flask_cors',
        'flask_restful', 'flask_restful.utils', 'flask_restful.representations',  # TODO ROSDEP has pip package
        'passlib', 'passlib.ext', 'passlib.ext.django', 'passlib.handlers', 'passlib.tests', 'passlib.utils', 'passlib.utils._blowfish', 'passlib._setup',  # TODO : rosdep has a DEB package for this
        'click',  # TODO : use deb package http://packages.ubuntu.com/search?keywords=python-click-cli ROSDEP also has python-click from pip
        'webargs',
        'marshmallow',
    ],
    package_dir={
        'rester': 'deps/Rester/rester',
        'testfixtures': 'deps/testfixtures/testfixtures',
        'flask_cors': 'deps/flask-cors/flask_cors',
        'flask_restful': 'deps/flask-restful/flask_restful',
        'passlib': 'deps/passlib/passlib',
        'click': 'deps/click/click',
        'webargs': 'deps/webargs/webargs',
        'marshmallow': 'deps/marshmallow/marshmallow',
    },
    entry_points={
        'console_scripts': [
            'rostful = rostful.__main__:cli'
        ]
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'futures == 3.0.2',
        'Flask==0.10.1',
        'Flask-Cors==2.0.1',
        'Flask-Script',
        'Flask-Restful',
        'Flask-reverse-proxy',
        'Rester',
        'click',
        'webargs',
        'pyros==0.1.0',
        'tornado == 4.0.2',
    ],
    test_suite="nose.collector",
    tests_require=[
        'nose>=1.3.7'
    ],
    zip_safe=False,  # TODO testing...
)

