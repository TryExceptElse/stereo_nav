from setuptools import setup, Extension
from Cython.Build import cythonize


setup(
    name="nav",
    ext_modules=cythonize([
        Extension(
            name='nav.optic_intf',
            sources=['nav/optic_intf.pyx'],
            extra_compile_args=['-O3', '-march=native', '-std=c11'],
            language='c',
        )
    ])
)
