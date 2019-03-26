from setuptools import setup, Extension
from Cython.Build import cythonize


setup(
    name="nav",
    ext_modules=cythonize([
        Extension(
            name='nav.optic_intf',
            sources=[
                'nav/optic_intf.pyx',
                'nav/clib/optic_intf.c',
                'nav/third_party/pipe.c',
            ],
            extra_compile_args=['-g', '-O0', '-march=native', '-std=c11'],
            libraries=['uvc'],
            language='c',
        )
    ])
)
