from setuptools import setup, find_packages


with open('README.rst') as f:
    long_description = ''.join(f.readlines())


setup(
    name='pyems',
    version='0.0.1',
    description='Simple interface for reading and writing to the EMS bus via an Arduino',
    long_description=long_description,
    author='Adam Podroužek',
    author_email='podroada@fit.cvut.cz',
    keywords='ems, arduino, interface',
    license='MIT',
    url='https://github.com/Adam95/pyems',
    packages=find_packages(),
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Information Technology',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Topic :: Software Development',
        'Topic :: Utilities',
    ],
    install_requires=[
        'pyserial'
    ],
    setup_requires=['pytest-runner'],
    tests_require=['pytest'],
    entry_points={
        # 'console_scripts': [
        #     'pyems = pyems:main',
        # ]
    },
    package_data={
        'pyems': ['assets/*']
    },
    zip_safe=False,
)
