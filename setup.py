#!/usr/bin/env python

from distutils.core import setup, Extension
import os.path

SOURCE_DIR = "drivers"
SOURCE_FILES = [
    "I2Cdev.cpp",
    "Ublox.cpp",
    "MPU9250.cpp",
    "MS5611.cpp",
    "PCA9685.cpp",
    "ADS1115.cpp",
    "MB85RC04.cpp",
    "RCin.cpp",
]
EXT_SOURCE_DIR = os.path.join(SOURCE_DIR, "ext")

mod = Extension("navio",
                sources = ["navio.cpp"] + [os.path.join(SOURCE_DIR, src) for src in SOURCE_FILES],
                include_dirs = [SOURCE_DIR, EXT_SOURCE_DIR],
                extra_compile_args = ["-std=c++0x", "-DNAVIO_DEBUG"],
                extra_link_args=["-lpigpio", "-lrt", "-lpthread"])

setup(name="navio",
      version="0.9.1",
      description="Python extension for the Navio shield for Raspberry Pi",
      ext_modules=[mod])
