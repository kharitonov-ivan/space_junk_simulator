#!/usr/bin/env python

from setuptools import setup

import os, subprocess

cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
cpp_files = ['/src/model.cpp', '/src/cpu/cpu_solver.cpp', '/src/python_package/python_wrapper.cpp']
input_cpp_files = [cur_path + path for path in cpp_files]

o_files = ['src/python_package/' +
           path for path in ['model.o', 'cpu_solver.o', 'python_package.o']]
des_o_files = [cur_path + '/' + path for path in o_files]


for cpp_file, o_file in zip(input_cpp_files, o_files):
    get_object_command = ['g++', '-c', '-std=c++14', '-fPIC',
                          cpp_file, '-o', o_file]
    subprocess.call([*get_object_command])

shared__library = cur_path + '/src/python_package/python_package.so'
subprocess.call(['g++', '-shared', '-std=c++14', '-W1',
                 '-o', shared__library, *des_o_files,'/src/python_package/gpu_solver'])

try:
    subprocess.call(['nvcc','-c', '/src/gpu/gpu_solver.cu', '-o','/src/python_package/gpu_solver', '-Xcompiler', '-fPIC'])
except:
    pass

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='space_junk_simulator',
    version='0.1',
    packages=[''],
    package_dir = {'': 'src/python_package'},
    url='',
    license='',
    author='ivan_kharitonov',
    author_email='',
    description=''
)
