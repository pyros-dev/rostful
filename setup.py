# This setup is usable by catkin, or on its own as usual python setup.py

_CATKIN = False
try:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup
    _CATKIN = True
except Exception, e:
    from setuptools import setup

# CAREFUL distutils and setuptools take different arguments and have different behaviors
if _CATKIN:  # using distutils : https://docs.python.org/2/distutils
    # fetch values from package.xml
    setup_args = generate_distutils_setup(
        packages=[
            'rostful',
            # This can create potential conflicts in install space,
            # If another package install the same pthon package dependency.
            # TODO : The proper solution is to create ThirdPartyRelease for these packages if needed or
            # TODO : Use existing ros package for these
            'rester',
            'testfixtures',
            'flask_cors',
            'flask_restful', 'flask_restful.utils', 'flask_restful.representations',  # TODO ROSDEP has pip package
            'flask_migrate',
            'flask_security',
            'flask_wtf', 'flask_wtf.recaptcha',
            'flask_sqlalchemy',
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
            'flask_migrate': 'deps/Flask-Migrate/flask_migrate',
            'flask_security': 'deps/flask-security/flask_security',
            'flask_wtf': 'deps/flask-wtf/flask_wtf',
            'flask_sqlalchemy': 'deps/flask-sqlalchemy/flask_sqlalchemy',
            'passlib': 'deps/passlib/passlib',
            'click': 'deps/click/click',
            'webargs': 'deps/webargs/webargs',
            'marshmallow': 'deps/marshmallow/marshmallow',
        },
        # CATKIN doesn't work well with those => symlink workarounds
        py_modules=[
            'flask_login',
            'flask_principal',
            'flask_mail',
        ],
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
    )
    setup(**setup_args)

else:  # using setuptools : http://pythonhosted.org/setuptools/
    # TODO : This as a third party package could be the ultimate goal...
    setup(name='rostful',
        version='0.0.7',
        description='REST API for ROS',
        url='http://github.com/asmodehn/rostful',
        author='AlexV',
        author_email='asmodehn@gmail.com',
        license='BSD',
        packages=['rostful'],
        # this is better than using package data ( since behavior is a bit different from distutils... )
        include_package_data=True,  # use MANIFEST.in during install.
        install_requires=[
            'rostful_node',
            'Flask==0.10.1',
            'Flask-Celery-Helper',
            'Flask-Cors==2.0.1',
            'Flask-Script',
            'Flask-Security==1.7.4',
            'Flask-Restful',
            'Flask-SQLAlchemy==2.0',
            # python requests...
        ],
        zip_safe=False,  # TODO testing...
    )
