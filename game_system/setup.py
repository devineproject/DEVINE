'''setup.py'''
from setuptools import setup

setup(
    name='game_system',
    version='1.0.0',
    packages=[''],
    package_dir={'': 'src'},
    url='',
    license='BSD License 2.0',
    author='DEVINE',
    author_email='',
    scripts=['bin/client.py'],
    description='Game system for DEVINE Guesswhat?!', install_requires=['transitions', 'enum34']
)
