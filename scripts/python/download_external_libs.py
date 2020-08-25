
from download_header_only import download_headers_only, HeaderOnlyDescription

# Disclaimer: This script is called by qmake or CMake. It is not necessary to call it manually.

# Downloads the external libs if they are not already there.
# Puts them in extern/*
# Only moves the given include directories.

# why is this not even trying eigen?

download_headers_only(( \
        HeaderOnlyDescription("eigen", "eigen-3.2.10", "eigen-3.2.10.zip", "https://www.gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.zip"), \
        HeaderOnlyDescription("cgal", "CGAL-5.0", "CGAL-5.0-library.zip", "https://www.github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0/CGAL-5.0-library.zip") \
     ))



