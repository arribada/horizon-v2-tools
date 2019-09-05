from __future__ import unicode_literals

import re

from setuptools import setup, find_packages


def get_version(filename):
    content = open(filename).read()
    metadata = dict(re.findall("__([a-z]+)__ = '([^']+)'", content))
    return metadata['version']


setup(
    name='arribada_tools',
    version=get_version('arribada_tools/__init__.py'),
    url='https://tbd',
    license='GNU General Public License v3 or later (GPLv3+)',
    author='Liam Wickins',
    author_email='liam@icoteq.com',
    description='Python tools for provisioning Arribada tracker devices',
    long_description=open('README.rst').read(),
    packages=find_packages(exclude=['tests', 'tests.*']),
    zip_safe=False,
    include_package_data=True,
    install_requires=[
        'setuptools',
        'pyusb >= 1.0.2',
        'pyserial >= 3.4',
        'bluepy >= 1.1.4',
        'python-dateutil >= 2.6.1',
        'libusb1 >= 1.6.4',
    ],
    test_suite='nose.collector',
    tests_require=[
        'nose',
        'mock >= 1.0',
    ],
    scripts=[
        'tests/tracker_config',
        'tests/gps_ascii_config',
        'tests/log_parse',
        'tests/ble_scan',
        'tests/ble_auto',
    ],
    classifiers=[
        'Development Status :: 6 - Alpha',
        'Intended Audience :: End Users/Desktop',
        'License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Topic :: Software Development :: Libraries',
        'Topic :: Communications',
    ],
)
