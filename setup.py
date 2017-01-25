#!/usr/bin/env python

from distutils.core import setup

setup(name='MIDCA',
      version='2.0',
      description='Metacognitive Integrated Dual-Cycle Architecture',
      author='Michael T. Cox and MIDCA Lab',
      author_email='wsri-midca-help@wright.edu',
      packages=['MIDCA',
                'MIDCA.domains',
                'MIDCA.domains.blocksworld',
                'MIDCA.domains.logistics',
                'MIDCA.domains.blocksworld.plan',
                'MIDCA.domains.nbeacons',
                'MIDCA.domains.nbeacons.plan',
                'MIDCA.domains.jshop_domains',
                'MIDCA.domains.jshop_domains.blocks_world',
                'MIDCA.domains.jshop_domains.logistics',
                'MIDCA.worldsim',
                'MIDCA.examples',
                'MIDCA.modules',
                'MIDCA.modules._adist',
                'MIDCA.modules._plan',
                'MIDCA.modules._plan.asynch',
                'MIDCA.modules._plan.jShop',
                'MIDCA.modules._goalgen',
                'MIDCA.modules._robot_world',
                'MIDCA.modules._xp_goal',
                'MIDCA.vision',
                'MIDCA.experimental',
                'MIDCA.experimental.baxter',
                'MIDCA.experiment',
                'MIDCA.metamodules'],
    
      package_data={'': ['*.jar', 'MIDCA.modules._plan.jShop'],
                    '': ['*.shp', 'MIDCA.domains.jshop_domains.blocks_world']},
      package_dir={"": ".."}
     )
