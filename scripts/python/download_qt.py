
from download_binary_lib import download_binary_libs, BinaryLibDescription

# Disclaimer: This script is called by qmake or CMake. It is not necessary to call it manually.

# Same as download_external_libs but only for qt.

download_binary_libs([BinaryLibDescription("qt", "qt.zip", "https://meshes.mailbase.info/libs/qt.zip")])
