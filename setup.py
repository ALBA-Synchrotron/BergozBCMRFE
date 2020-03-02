#!/usr/bin/env python

from setuptools import setup

# managed by bumpversion do not edit manually
__version = '1.0.0'


def main():

    setup(
        name='bergozbcmrfe',
        version=__version,
        package_dir={'bergozbcmrfe': 'src'},
        packages=['bergozbcmrfe'],
        include_package_data=True,  # include files in MANIFEST
        author='Jairo Moldes',
        author_email='jmoldes@cells.es',
        description='Tango device server for the BergozBCM-RF-E instrument',
        license='GPLv3+',
        platforms=['src', 'noarch'],
        url='https://github.com/ALBA-Synchrotron/BergozBCMRFE',
        install_requires=['tango', 'pyserial'],
        entry_points={
            'console_scripts': [
                'BergozBCMRFE = bergozbcmrfe.BergozBCMRFE:main',
            ],
        },
    )


if __name__ == "__main__":
    main()
