
import ctypes
import numpy
import glob
import os
from ctypes import *

class Object(ctypes.Structure):

    _fields_ = [('x', ctypes.POINTER(ctypes.c_double)),
                ('y', ctypes.POINTER(ctypes.c_double)),
                ('z', ctypes.POINTER(ctypes.c_double)),
                ('vx', ctypes.POINTER(ctypes.c_double)),
                ('vy', ctypes.POINTER(ctypes.c_double)),
                ('vz', ctypes.POINTER(ctypes.c_double)),
                ('size', ctypes.POINTER(ctypes.c_double)),
               ]

class space_simulator:
    def __init__(self, gpu = False):
        print(os.getcwd())
        cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        print(cur_path)
        so_cpu_path = cur_path + '/python_package_cpu.so'
        self.cpu_lib = ctypes.CDLL(so_cpu_path)
        self.cpu_lib.solve_cpu.argtypes = [numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # x
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # y
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # z
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vx
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vy
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vz
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # x_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # y_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # z_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vx_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vy_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vz_res
                                              c_size_t,
                                              c_size_t,
                                              c_double]

        self.cpu_lib.solve_cpu.restype = None
        print("Ok!")

        if gpu:
            so_gpu_path =  cur_path + '/python_package_gpu.so'
            self.gpu_lib = ctypes.CDLL(so_gpu_path)
            self.gpu_lib.solve_gpu.argtypes = [numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # x
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # y
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # z
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vx
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vy
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vz
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # x_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # y_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # z_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vx_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vy_res
                                              numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # vz_res
                                              c_size_t,
                                              c_double]

#             self.gpu_lib.solve_gpu.restype = None

    def run(self, x, y, z, vx, vy, vz, size, vzsteps, timestep, gpu = False):
        x_res, y_res, z_res, vx_res, vy_res, vz_res = numpy.zeros_like([x, y, z, vx, vy, vz])

        if gpu==False:
            print(x, y, z, vx, vy, vz)
            self.cpu_lib.solve_cpu(x, y, z, vx, vy, vz, x_res, y_res, z_res, vx_res, vy_res, vz_res, size, vzsteps, timestep)
            print(x_res, y_res, z_res, vx_res, vy_res, vz_res)
        else:
            self.gpu_lib.solve_gpu(x, y, z, vx, vy, vz, x_res, y_res, z_res, vx_res, vy_res, vz_res, vzsteps, timestep)

 
        return x_res, y_res, z_res, vx_res, vy_res, vz_res



