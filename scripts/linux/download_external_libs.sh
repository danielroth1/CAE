#!/bin/bash

# Disclaimer: This script is called by qmake or CMake. It is not necessary to call it manually.

# Downloads the external libs if they are not already there.
# Puts them in extern/*
# Only moves the given include directories.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

bash $DIR/download_header_only.sh "cgal" "CGAL-5.0" "CGAL-5.0-library.zip" "https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0/CGAL-5.0-library.zip"
bash $DIR/download_header_only.sh "eigen" "eigen-3.2.10" "eigen-3.2.10.zip" "https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.zip"


