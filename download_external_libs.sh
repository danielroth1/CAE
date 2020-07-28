#!/bin/bash

extern_folder="extern"

debug_output=false

# create a tmp folder that doesn't exist yet. Simply appends _
tmp_folder="_tmp"
while [ -d $tmp_folder ]; do
    tmp_folder="_$tmp_folder"
done

if [ ! -d $extern_folder ]; then
    mkdir $extern_folder
fi

download_header_only()
{
    target_folder=$1 # "cgal"
    include_folder=$2 # "CGAL-5.0.2/include"
    zip_file=$3 # "CGAL-5.0.2-library.zip"
    download_link=$4 # "https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2-library.zip"

    if [ -d $extern_folder/$target_folder ]; then
        if [ $debug_output == true ]; then # This message is really annoying.
            echo "$target_folder already exists. Skipping..."
        fi
    else
        mkdir -p $tmp_folder
        cd $tmp_folder
	echo "$target_folder not found, downloading..."
        wget $download_link
	echo "Downloading complete."
        unzip $zip_file &> /dev/null
        rm -r $zip_file
        mkdir -p ../$extern_folder/cgal
        mv $include_folder ../extern/$target_folder
	echo "Successfully downloaded and installed $target_folder."
        cd ..
    fi
}

download_header_only "cgal" "CGAL-5.0.2/include" "CGAL-5.0.2-library.zip" "https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2-library.zip"

download_header_only "eigen" "eigen-3.2.10" "eigen-3.2.10.zip" "https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.zip"

if [ -d $tmp_folder ]; then 
    rm -r $tmp_folder
fi


