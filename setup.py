from setuptools import setup, find_packages
from setuptools.command.build_ext import build_ext
from setuptools.command.build_py import build_py
import os
import sys
import subprocess

class CMakeExtension:
    def __init__(self, name):
        self.name = name

class CMakeBuild(build_ext):
    def run(self):
        for ext in self.extensions:
            self.build_cmake(ext)

    def build_cmake(self, ext):
        cwd = os.path.abspath(os.path.dirname(__file__))
        build_temp = os.path.join(cwd, 'build')
        
        if not os.path.exists(build_temp):
            os.makedirs(build_temp)

        subprocess.check_call(['cmake', cwd], cwd=build_temp)
        subprocess.check_call(['cmake', '--build', '.'], cwd=build_temp)

setup(
    name="TwinRehabAlgorithm",
    version="1.0.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="Digital Twin System for Musculoskeletal Rehabilitation",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20",
        "scipy>=1.7",
        "pandas>=1.3",
        "vtk>=9.0",
        "matplotlib>=3.4",
        "scikit-learn>=0.24",
        "pytest>=6.0",
        "pytest-cov>=2.12",
    ],
    ext_modules=[CMakeExtension("biomech")],
    cmdclass={
        "build_ext": CMakeBuild,
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
)