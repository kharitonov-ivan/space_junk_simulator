#!/usr/bin/env python
from setuptools import setup
import os, subprocess
import argparse


def cpu_version_build(cur_path, sanitaser=False):
    cpp_files = ['/src/cpu/cpu_solver.cpp',
                 '/src/python_package/py_wrapper_cpu.cpp',
                 '/src/model.cpp']

    # Build only CPU version

    sanitaser_setup = ['-Wall', '-Werror']
    input_cpp_files = [cur_path + path for path in cpp_files]

    o_files = [cur_path + '/src/python_package/build/' + path
               for path in ['cpu_solver.o', 'py_wrapper_cpu.o', 'model_cpu.o']]
    print(cpp_files)

    for cpp_file, o_file in zip(input_cpp_files, o_files):
        get_object_command = ['g++', '-c', '-std=c++14','-fPIC',
                              cpp_file, '-o', o_file]
        if sanitaser:
            get_object_command += sanitaser_setup

        subprocess.call([*get_object_command])

    so_cpu_path = cur_path + '/src/python_package/build/python_package_cpu.so'
    subprocess.call(['g++', '-shared', '-std=c++14', '-o', so_cpu_path, *o_files])


def gpu_version_build(cur_path):
    # Build only GPU version
    so_gpu_path = cur_path + '/src/python_package/build/python_package_gpu.so'
    
    subprocess.call(['g++', '-c', '-std=c++14',  
                     cur_path + '/src/python_package/py_wrapper_gpu.cpp', 
                     '-o', 
                     cur_path + '/src/python_package/build/py_wrapper_gpu.o',
                     '-fPIC'])
    subprocess.call(['nvcc', '-c', '-std=c++14',
                     cur_path + '/src/gpu/gpu_solver.cu',
                     '-o',
                     cur_path +'/src/python_package/build/gpu_solver.o',
                     '-Xcompiler',
         '-fPIC'])

    subprocess.call(['nvcc', '-c', '-std=c++14',
                     cur_path + '/src/model.cpp',
                     '-o',
                     cur_path +'/src/python_package/build/model_cuda.o',
                     '-Xcompiler',
                     '-fPIC'])

    subprocess.call(['nvcc', '-shared', '-std=c++14',
                     '-o', so_gpu_path,
                     cur_path + '/src/python_package/build/model_cuda.o',
                     cur_path + '/src/python_package/build/py_wrapper_gpu.o',
                     cur_path + '/src/python_package/build/gpu_solver.o'])
    print("GPU OK", so_gpu_path)


SANITASER = False
cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser()
parser.add_argument("--GPU", help="Build GPU module", default = None)
args = parser.parse_args()

if args.GPU:
    print("Build GPU version")
    cpu_version_build(cur_path)
    gpu_version_build(cur_path)

else:
    print("Build CPU version")
    cpu_version_build(cur_path)





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
