#!/bin/bash

acceptable_file_types=(".obj" ".mtl" ".off" ".node" ".ele" ".tet" ".bmp" ".png")
folder="assets"
server="https://meshes.mailbase.info"

debug_output=false

# Check if a file exists with one of the endings in acceptable_file_types and if not
# downloads the corresponding zip file from the specified server, upacks it, and removes the .zip again.
# params:
# file_path - FEMFX/car-body-tets
download_file_if_not_exists()
{
    file_path=$1

    if [ ! -d $folder ]; then
        mkdir $folder
    fi

    # search for file name
    
    # split path up
    # path_list = [FEMFX car-body-tets]
    del="/"
    IFS='/' read -ra path_list <<< "$file_path"

    # e.g.
    # path_to_file = FEMFX
    # file_name = car-body-tets
    path_to_file=""
    if [[ ${#path_list[@]} > 1 ]]; then
        path_to_file="${path_list[0]}"
    fi
    file_name="${path_list[$(expr ${#path_list[@]} - 1)]}"
    for i in "${!path_list[@]}"; do # iterate with indices
        if [[ $i != 0 && $i < $(expr ${#path_list[@]} - 1) ]]; then
            path_to_file="$path_to_file/${path_list[$i]}"
        fi
    done

    if [ $debug_output == true ]; then
        echo "path to file = $path_to_file"
        echo "file_name = $file_name"
    fi

    # check if the file exists by trying out all supported extensions
    # check for:
    # FEMFX/car-body-tets.obj
    # FEMFX/car-body-tets.mtl
    # FEMFX/car-body-tets.off
    # etc.
    existing_file=""
    for i in "${acceptable_file_types[@]}"; do
        checked_file="$folder/$file_path$i"
        if [ -e "$checked_file" ]; then
            existing_file="$checked_file"
            break
        fi
    done

    if [ $debug_output == true ]; then
        echo "existing_file = $existing_file"
    fi

    if [ "$existing_file" != "" ]; then
        if [ $debug_output == true ]; then # This message is really annoying.
            echo "File $existing_file is already there. Done."
        fi
        return
    fi
    
    # file_to_download = https://meshes.mailbase.info/FEMFX/car-body-tets.zip
    file_to_download="$file_path.zip"
    echo "File $file_path does not exist. Downloading file $file_to_download from $server..."

    # zip_file = car-body-tets.zip
    zip_file="$file_name.zip"

    # https://mailbase.info/meshes/car-body-tets.zip
    download_link="$server/$file_to_download"

    # create _tmp folder
    mkdir -p $tmp_folder
    cd $tmp_folder

    # download zip and unzip
    wget $download_link
    echo "Download complete, unzip..."
    unzip $zip_file &> /dev/null

    # remove unnecessary files
    rm $zip_file # remove car-body-tets.zip
    if [ -f "LICENSE.txt" ]; then 
        rm "LICENSE.txt" # remove any license files that were also in the .zip
    fi

    # create target folder and move all files to there
    mkdir -p "../$folder/$path_to_file/"
    shopt -s dotglob nullglob
    mv * "../$folder/$path_to_file/"

    echo "Installation of $file_path is done."
    cd ..
}

# Create there is already a _tmp folder, remove all files in it.
tmp_folder="_tmp"
while [ -d $tmp_folder ]; do
    tmp_folder="_$tmp_folder"
done

if [ $debug_output == true ]; then
    echo "tmp_folder = $tmp_folder"
fi

for var in "$@"
do
    download_file_if_not_exists $var
done

if [ -d $tmp_folder ]; then 
    rm -r $tmp_folder
fi


