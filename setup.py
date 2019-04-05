
#!/usr/bin/env python
from setuptools import setup
import os, subprocess

USE_GPU = True

cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))

cpp_files = ['/src/model.cpp', '/src/cpu/cpu_solver.cpp', '/src/python_package/py_wrapper_cpu.cpp', '/src/python_package/py_wrapper_gpu.cpp']
input_cpp_files = [cur_path + path for path in cpp_files]

o_files = [cur_path + '/src/python_package/' + path
           for path in ['model.o', 'cpu_solver.o', 'python_package.o']]

for cpp_file, o_file in zip(input_cpp_files, o_files):
    get_object_command = ['g++', '-c', '-std=c++14', '-fPIC',
                          cpp_file, '-o', o_file]
    subprocess.call([*get_object_command])


so_cpu_path = cur_path + '/src/python_package/python_package_cpu.so'
so_gpu_path = cur_path + '/src/python_package/python_package_gpu.so'

subprocess.call(['g++', '-shared', '-std=c++14', '-o', so_cpu_path, *o_files])

if USE_GPU:
    subprocess.call(
        ['nvcc', '-c', '-std=c++14', cur_path + '/src/gpu/gpu_solver.cu', '-o', cur_path +'/src/python_package/gpu_solver.o', '-Xcompiler',
         '-fPIC'])

    subprocess.call(['g++', '-shared', '-std=c++14',
                     '-o', so_gpu_path, *o_files, cur_path + '/src/python_package/gpu_solver.o'])
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
