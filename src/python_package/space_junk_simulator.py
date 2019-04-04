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
    def __init__(self):
        #print(os.getcwd())
        cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        libfile = ('/python_package_cpu.so')
        self.solver_cpu = ctypes.CDLL(cur_path + libfile)
        self.solver_cpu.main_.argtypes = [numpy.ctypeslib.ndpointer(dtype=numpy.float64, flags="C_CONTIGUOUS"),  # x
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

        self.solver_cpu.main_.restype = None
        print("Ok!")

    def run(self, x, y, z, vx, vy, vz, vzsteps=10, timestep = 1.0):
        x_res, y_res, z_res, vx_res, vy_res, vz_res = x, y, z, vx, vy, vz
        self.solver_cpu.main_(x, y, z, vx, vy, vz, x_res, y_res, z_res, vx_res, vy_res, vz_res, vzsteps, timestep)
        print("run!")
        return x_res, y_res, z_res, vx_res, vy_res, vz_res



