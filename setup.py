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
    packages=['rostful'],
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

