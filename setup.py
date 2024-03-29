#! /usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup
from setuptools_rust import Binding, RustExtension

setup(
    name='caterpillar',
    version='0.1.6',
    author='Matthew Ishige',
    rust_extensions=[
        RustExtension('caterpillar_lib.caterpillar', 'lib/Cargo.toml', binding=Binding.RustCPython)
    ],
    packages=['caterpillar_lib'],
    zip_safe=False
)
