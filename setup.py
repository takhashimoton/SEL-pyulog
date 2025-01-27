#!/usr/bin/env python
"""Python log parser for ULog.

This module allows you to parse ULog files, which are used within the PX4
autopilot middleware.

The file format is documented on https://docs.px4.io/master/en/dev_log/ulog_file_format.html

"""

from __future__ import print_function
import os
import sys
import versioneer

from setuptools import setup, find_packages

DOCLINES = __doc__.split("\n")

CLASSIFIERS = """\
Development Status :: 1 - Planning
Intended Audience :: Science/Research
Intended Audience :: Developers
License :: OSI Approved :: BSD License
Programming Language :: Python
Programming Language :: Python :: 3
Programming Language :: Other
Topic :: Software Development
Topic :: Scientific/Engineering :: Artificial Intelligence
Topic :: Scientific/Engineering :: Mathematics
Topic :: Scientific/Engineering :: Physics
Operating System :: Microsoft :: Windows
Operating System :: POSIX
Operating System :: Unix
Operating System :: MacOS
"""

# pylint: disable=invalid-name

setup(
    name='pyulog-sel',
    maintainer="Takenori Hashimoto",
    maintainer_email="takenori.hashimoto@selab.jp",
    description=DOCLINES[0],
    long_description="\n".join(DOCLINES[2:]),
    url='https://github.com/takhashimoton/SEL-pyulog.git',
    author='Beat Kueng',
    author_email='beat-kueng@gmx.net',
    download_url='https://github.com/takhashimoton/SEL-pyulog.git',
    license='BSD 3-Clause',
    classifiers=[_f for _f in CLASSIFIERS.split('\n') if _f],
    platforms=["Windows", "Linux", "Solaris", "Mac OS-X", "Unix"],
    install_requires=['numpy'],
    tests_require=['pytest', 'ddt'],
    entry_points = {
        'console_scripts': [
            'ulog_extract_gps_dump=pyulog.extract_gps_dump:main',
            'ulog_info=pyulog.info:main',
            'ulog_messages=pyulog.messages:main',
            'upc=pyulog.params:main',
            'ulog2csv=pyulog.ulog2csv:main',
            'ulog2kml=pyulog.ulog2kml:main',
            'ulog2rosbag=pyulog.ulog2rosbag:main',
            'ulog_migratedb=pyulog.migrate_db:main',
        ],
    },
    packages=find_packages(),
    version='1.0.11',
    cmdclass=versioneer.get_cmdclass(),
    include_package_data=True,
)
