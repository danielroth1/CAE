#!/bin/bash

# Creates an AppImage using linuxdeployqt, see https://github.com/probonopd/linuxdeployqt.
# Requires the program to be build.
# \param 1 - path to the folder that contains the executable "CAE" and the assets/ folder. This is usually your release or profile build directory.

# Example call: ./create_appimage.sh ../../build-CAE-Desktop_Qt_5_9_5_GCC_64bit-Profile

executable_path=$1/CAE
assets_path=$1/assets

if [ ! -f $executable_path ]; then
    echo "Error: Executable does not exist."
    exit 1
fi

# $DIR is the path to the folder in which this script lies.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

appimage_folder=$DIR/appimage
mkdir -p $appimage_folder

# TODO: it seems like icon support isn't implemented with native AppImages, see https://github.com/AppImage/AppImageKit/issues/346
# So, this doesn't work right now but I let the code here if there is a solution in the future. The logo is still needed to build the AppImage
# because it always needs to be specified one in the default.desktop

# download logo to assets/ and copy it to appimage_folder
bash $DIR/../scripts/linux/download_file_if_not_exists.sh "logo"
cp $DIR/assets/logo.png $appimage_folder/

# download linuxdeployqt to extern/
if [ ! -f "extern/linuxdeployqt-6-x86_64.AppImage" ]; then
    mkdir -p extern
    cd extern
    wget https://github.com/probonopd/linuxdeployqt/releases/download/6/linuxdeployqt-6-x86_64.AppImage
    chmod a+x linuxdeployqt-6-x86_64.AppImage
    cd ..
fi

# copy executable to appimage_folder
cp $executable_path $appimage_folder

# copy default.desktop to appimage_folder
cp $DIR/default.desktop $appimage_folder

# copy assets folder
cp -r $assets_path $appimage_folder

# create app image
cd $appimage_folder

../extern/linuxdeployqt-6-x86_64.AppImage ./CAE -appimage -unsupported-allow-new-glibc -extra-plugins=iconengines


