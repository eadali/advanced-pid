#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 20:51:46 2022

@author: eadali
"""

from distutils.core import setup
setup(
  name = 'advanced-pid',
  packages = ['advanced_pid', 'advanced_pid.integrate', 'advanced_pid.models'],
  version = '0.0.4',
  license='MIT',
  description = 'An advanced PID controller in Python.',
  long_description='An advanced PID controller in Python. The derivative term can also be used in real applications thanks to built-in first-order filter.',
  author = 'Erkan ADALI',
  author_email = 'erkanadali91@gmail.com',
  url = 'https://github.com/eadali/advanced-pid',
  project_urls={'Documentation': 'https://advanced-pid.readthedocs.io/',
                'Source Code': 'https://github.com/eadali/advanced-pid',},
  keywords = ['control', 'theory', 'engineering', 'pid', 'real', 'time',],
  install_requires=['numpy',],
  classifiers=[
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3',
  ],
)