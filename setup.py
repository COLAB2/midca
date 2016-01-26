#!/usr/bin/env python

from distutils.core import setup

setup(name='MIDCA',
      version='2.0',
      description='Metacognitive Integrated Dual-Cycle Architecture',
      author='Matt Paisner',
      author_email='matthew.paisner@gmail.com',
      packages=['MIDCA', 'MIDCA.worldsim', 'MIDCA.examples', 'MIDCA.modules', 'MIDCA.vision','MIDCA.modules._adist', 'MIDCA.modules._plan', 'MIDCA.modules._plan.asynch', 'MIDCA.modules._goalgen', 'MIDCA.modules._robot_world', 'MIDCA.experimental', 'MIDCA.experimental.baxter'],
      package_dir={"": ".."}
     )
