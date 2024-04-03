from distutils.core import setup, Extension
setup(name='spicon', version='1.0',  \
      ext_modules=[Extension('spicon', ['libspi.c'])])
