from setuptools import setup

# # CAREFUL distutils and setuptools take different arguments and have different behaviors
# if _CATKIN:  # using distutils : https://docs.python.org/2/distutils
#     # fetch values from package.xml
#     setup_args = generate_distutils_setup(
#         packages=[
#             'rostful',
#             'rostful.api_0_1',
#             'rostful.api_0_2',
#             'rostful.frontend',
#             'rostful.tests',
#             # This can create potential conflicts in install space,
#             # If another package install the same pthon package dependency.
#             # TODO : The proper solution is to create ThirdPartyRelease for these packages if needed or
#             # TODO : Use existing ros package for these
#             # 'rester',
#             # 'testfixtures',
#             'flask_cors',
#             'flask_reverse_proxy',
#             'flask_restful', 'flask_restful.utils', 'flask_restful.representations',  # TODO ROSDEP has pip package
#             # 'passlib', 'passlib.ext', 'passlib.ext.django', 'passlib.handlers', 'passlib.tests', 'passlib.utils', 'passlib.utils._blowfish', 'passlib._setup',  # TODO : rosdep has a DEB package for this
#             'click',  # TODO : use deb package http://packages.ubuntu.com/search?keywords=python-click-cli ROSDEP also has python-click from pip
#             'webargs',
#             'marshmallow',
#         ],
#         package_dir={
#             # 'rester': 'deps/Rester/rester',
#             #'testfixtures': 'deps/testfixtures/testfixtures',
#             'flask_cors': 'deps/flask-cors/flask_cors',
#             'flask_reverse_proxy': 'deps/flask-reverse-proxy/flask_reverse_proxy',
#             'flask_restful': 'deps/flask-restful/flask_restful',
#             #'passlib': 'deps/passlib/passlib',
#             'click': 'deps/click/click',
#             'webargs': 'deps/webargs/webargs',
#             'marshmallow': 'deps/marshmallow/marshmallow',
#         },
#         # CATKIN doesn't work well with those => symlink workarounds
#         # py_modules=[
#         #     'flask_login',
#         # ],
#         package_data={
#             'rostful': [
#                 'static/favicon.ico',
#                 'static/js/moment/*',
#                 'static/js/jquery/*',
#                 'static/js/jquery-mobile/*',
#                 'static/js/jquery-mobile/images/*',
#                 'static/js/jquery-mobile/images/icons-png/*',
#                 'static/js/jquery-mobile/images/icons-svg/*',
#                 'templates/*',
#                 'templates/security/*',
#                 'templates/security/email/*',
#             ],
#         },
#     )
#     setup(**setup_args)
#
# else:

# using setuptools : http://pythonhosted.org/setuptools/
# TODO : Use the bdist_wheel setuptools extension available from the wheel project to create wheels.
# TODO : This as a third party package could be the ultimate goal...

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
        'rostful.api_0_1',
        'rostful.api_0_2',
        'rostful.frontend',
        'rostful.tests',
        # This can create potential conflicts in install space,
        # If another package install the same pthon package dependency.
        # TODO : The proper solution is to create ThirdPartyRelease for these packages
        # 'rester',
        #'testfixtures',
        #'flask_cors',
        #'flask_restful', 'flask_restful.utils', 'flask_restful.representations',  # TODO ROSDEP has pip package
        #'flask_reverse_proxy',
        # 'passlib', 'passlib.ext', 'passlib.ext.django', 'passlib.handlers', 'passlib.tests', 'passlib.utils', 'passlib.utils._blowfish', 'passlib._setup',  # TODO : rosdep has a DEB package for this
        #'click',  # TODO : use deb package http://packages.ubuntu.com/search?keywords=python-click-cli ROSDEP also has python-click from pip
        #'webargs',
        #'marshmallow',
    ],
    package_dir={
        # 'rester': 'deps/Rester/rester',
        #'testfixtures': 'deps/testfixtures/testfixtures',
        'flask_cors': 'deps/flask-cors/flask_cors',
        'flask_reverse_proxy': 'deps/flask-reverse-proxy/flask_reverse_proxy',
        'flask_restful': 'deps/flask-restful/flask_restful',
        # 'passlib': 'deps/passlib/passlib',
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
    package_data={
        'rostful': [
            'static/favicon.ico',
            'static/js/moment/*',
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
    install_requires=[
        'futures>=3.0.2',
        'Flask>=0.10.1',
        'Flask-Cors>=3.0.2',
        #'Flask-Script',
        'Flask-Restful<=0.3.4',
        'Flask-reverse-proxy',
        #'Rester',
        'click>=6.4.0',
        'webargs>=1.3.4',
        'pyros>=0.3.0',
        'tornado>=4.2.1',
        'simplejson',
    ],
    test_suite="nose.collector",
    tests_require=[
        'nose>=1.3.7'
    ],
    zip_safe=False,  # TODO testing...
)

