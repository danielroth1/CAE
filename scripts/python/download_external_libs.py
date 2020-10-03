
from download_header_only import download_headers_only, HeaderOnlyDescription
from download_binary_lib import download_binary_libs, BinaryLibDescription

# Disclaimer: This script is called by qmake or CMake. It is not necessary to call it manually.

# Downloads the external libs if they are not already there.
# Puts them in extern/*
# Only moves the given include directories.

download_headers_only(( \
        HeaderOnlyDescription("eigen", "eigen-3.2.10", "eigen-3.2.10.zip", "https://www.gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.zip"), \
        HeaderOnlyDescription("cgal", "CGAL-5.0", "CGAL-5.0-library.zip", "https://www.github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0/CGAL-5.0-library.zip") \
     ))

download_binary_libs((
        BinaryLibDescription("freeglut", "freeglut.zip", "https://meshes.mailbase.info/libs/freeglut.zip"), \
        BinaryLibDescription("glew", "glew.zip", "https://meshes.mailbase.info/libs/glew.zip"), \
        BinaryLibDescription("gmp", "gmp.zip", "https://meshes.mailbase.info/libs/gmp.zip") \
))

