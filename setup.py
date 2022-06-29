#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 20:51:46 2022

@author: eadali
"""

from setuptools import setup, find_packages
from codecs import open
from os import path


here = path.abspath(path.dirname(__file__))
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()
    
setup(
  name = 'advanced-pid',
  version = '0.0.8',
  long_description=long_description,
  long_description_content_type='text/markdown',
  url = 'https://github.com/eadali/advanced-pid',
  author = 'eadali',
  license='MIT',
  classifiers=[
    'Development Status :: 4 - Beta',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3',
  ],
  keywords = ['control theory', 'control', 'theory', 'engineering', 'pid', 'real', 'time',],
  packages = ['advanced_pid', 'advanced_pid.models'],
  project_urls={'Documentation': 'https://advanced-pid.readthedocs.io/',
                'Source Code': 'https://github.com/eadali/advanced-pid',},
)
