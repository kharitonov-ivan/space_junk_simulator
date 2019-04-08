#!/usr/bin/env python
from setuptools import setup
import os, subprocess
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--GPU", help="Build GPU module")
args = parser.parse_args()
if args.GPU:
    print("GPU Mode")


USE_GPU = args.GPU
SANITASER = False

cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))


cpp_files = ['/src/model.cpp', 
             '/src/cpu/cpu_solver.cpp']

# Build only CPU version

sanitaser_setup = ['-Wall', '-Werror']
input_cpp_files = [cur_path + path for path in cpp_files + ['/src/python_package/py_wrapper_cpu.cpp']]
o_files = [cur_path + '/src/python_package/' + path
           for path in ['model.o', 'cpu_solver.o', 'py_wrapper_cpu.o']]

for cpp_file, o_file in zip(input_cpp_files, o_files):
    get_object_command = ['g++', '-c', '-std=c++14','-fPIC',
                          cpp_file, '-o', o_file]
    if SANITASER:
        get_object_command += sanitaser_setup
    subprocess.call([*get_object_command])

so_cpu_path = cur_path + '/src/python_package/python_package_cpu.so'
subprocess.call(['g++', '-shared', '-std=c++14', '-o', so_cpu_path, *o_files])


if USE_GPU:
    # Build only GPU version
    so_gpu_path = cur_path + '/src/python_package/python_package_gpu.so'
    
    subprocess.call(['g++', '-c', '-std=c++14',  
                     cur_path + '/src/python_package/py_wrapper_gpu.cpp', 
                     '-o', 
                     cur_path + '/src/python_package/py_wrapper_gpu.o',
         '-fPIC'])
    subprocess.call(['nvcc', '-c', '-std=c++14', 
                     cur_path + '/src/gpu/gpu_solver.cu', '-o', 
                     cur_path +'/src/python_package/gpu_solver.o', '-Xcompiler',
         '-fPIC'])
    subprocess.call(['nvcc', '-shared', '-std=c++14',
                     '-o', so_gpu_path, 
                     cur_path + '/src/python_package/model.o', 
                     cur_path + '/src/python_package/py_wrapper_gpu.o', 
                     cur_path + '/src/python_package/gpu_solver.o'])
    print("GPU OK", so_gpu_path)





# setup(
#     name='space_junk_simulator',
#     version='0.1',
#     packages=[''],
#     package_dir={'': 'src/python_package'},
#     url='',
#     license='',
#     author='ivan_kharitonov',
#     author_email='',
#     description=''
# )
