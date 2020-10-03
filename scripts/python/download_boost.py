
from download_binary_lib import download_binary_libs, BinaryLibDescription

# Disclaimer: This script is called by qmake or CMake. It is not necessary to call it manually.

# Same as download_external_libs but only for boost.

download_binary_libs([BinaryLibDescription("boost", "boost.zip", "https://meshes.mailbase.info/libs/boost.zip")])
