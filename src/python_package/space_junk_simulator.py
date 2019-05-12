import ctypes
import numpy as np
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
    def __init__(self, gpu=False):
        cur_path = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        self.gpu = gpu
        if self.gpu == False:
            so_cpu_path = cur_path + '/build/python_package_cpu.so'
            self.cpu_lib = ctypes.CDLL(so_cpu_path)
            self.cpu_lib.solve_cpu.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # x
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # y
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # z
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vx
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vy
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vz
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # x_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # y_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # z_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vx_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vy_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vz_res
                                               c_size_t,
                                               c_size_t,
                                               c_double]
            self.cpu_lib.solve_cpu.restype = None


        if self.gpu:
            so_gpu_path = cur_path + '/build/python_package_gpu.so'
            self.gpu_lib = ctypes.CDLL(so_gpu_path)
            self.gpu_lib.solve_gpu.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # x
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # y
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # z
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vx
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vy
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vz
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # x_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # y_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # z_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vx_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vy_res
                                               np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS"),  # vz_res
                                               c_size_t,
                                               c_size_t,
                                               c_double]
            self.gpu_lib.solve_gpu.restype = None


    def run(self, x, y, z, vx, vy, vz, size, vzsteps, timestep):
        x_res, y_res, z_res, vx_res, vy_res, vz_res = np.zeros_like([x, y, z, vx, vy, vz])

        if self.gpu == False:
            self.cpu_lib.solve_cpu(x, y, z, vx, vy, vz, x_res, y_res, z_res, vx_res, vy_res, vz_res, size, vzsteps,
                                   timestep)
        else:
            self.gpu_lib.solve_gpu(x, y, z, vx, vy, vz, x_res, y_res, z_res, vx_res, vy_res, vz_res, size, vzsteps,
                                   timestep)
        return x_res, y_res, z_res, vx_res, vy_res, vz_res


    def python_checker(self, x0, y0, z0, vx0, vy0, vz0, size, vzsteps, timestep, gpu=False):
        G = 6.67408e-11
        M = 5.972e24


        def get_accels(x0, y0, z0, vx0, vy0, vz0, ax0=0, ay0=0, az0=0, time=0, forces='simple_gravity'):
            forces_set = ['simple_gravity']
            ax, ay, az = 0, 0, 0
            if forces in forces_set:
                radius = np.sqrt(x0 ** 2 + y0 ** 2 + z0 ** 2)
                gravity_part = (G * M) / (radius ** 3)
                ax, ay, az = [- gravity_part * vector for vector in [x0, y0, z0]]
            return ax, ay, az

        dt = timestep * 2.0
        dt2 = timestep  # half of timestep

        ax0, ay0, az0 = get_accels(x0, y0, z0, vx0, vy0, vz0)

        # RK4 Calc k1, l1 vars
        kx1 = ax0 * dt2
        ky1 = ay0 * dt2
        kz1 = az0 * dt2

        lx1 = vx0 * dt2
        ly1 = vy0 * dt2
        lz1 = vz0 * dt2

        ax1, ay1, az1 = get_accels(x0 + lx1, y0 + ly1, z0 + lz1, vx0 + kx1,
                                   vy0 + ky1, vz0 + kz1)

        # RK4 Calc k2, l2  vars
        kx2 = ax1 * dt2
        ky2 = ay1 * dt2
        kz2 = az1 * dt2

        lx2 = (vx0 + kx1) * dt2
        ly2 = (vy0 + ky1) * dt2
        lz2 = (vz0 + kz1) * dt2

        ax2, ay2, az2 = get_accels(x0 + lx2, y0 + ly2, z0 + lz2, vx0 + kx2,
                                   vy0 + ky2, vz0 + kz2)

        # RK4 Calc k3, l3  vars
        kx3 = ax2 * dt
        ky3 = ay2 * dt
        kz3 = az2 * dt

        lx3 = (vx0 + kx2) * dt
        ly3 = (vy0 + ky2) * dt
        lz3 = (vz0 + kz2) * dt

        ax3, ay3, az3 = get_accels(x0 + lx3, y0 + ly3, z0 + lz3, vx0 + kx3,
                                   vy0 + ky3, vz0 + kz3)

        # RK4 Calc k3, l3  vars
        kx4 = ax3 * dt2
        ky4 = ay3 * dt2
        kz4 = az3 * dt2

        lx4 = (vx0 + kx3) * dt2
        ly4 = (vy0 + ky3) * dt2
        lz4 = (vz0 + kz3) * dt2

        x_step = x0 + (lx1 + 2.0 * lx2 + lx3 + lx4) / 3.0
        y_step = y0 + (ly1 + 2.0 * ly2 + ly3 + ly4) / 3.0
        z_step = z0 + (lz1 + 2.0 * lz2 + lz3 + lz4) / 3.0

        vx_step = vx0 + (kx1 + 2.0 * kx2 + kx3 + kx4) / 3.0
        vy_step = vy0 + (ky1 + 2.0 * ky2 + ky3 + ky4) / 3.0
        vz_step = vz0 + (kz1 + 2.0 * kz2 + kz3 + kz4) / 3.0

        return x_step, y_step, z_step, vx_step, vy_step, vz_step
