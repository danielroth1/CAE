#!/bin/bash

import os
import requests
import shutil
import system_helper
import zipfile
from pathlib import Path

debug_output = False

'''
Check if a file exists with one of the endings in acceptable_file_types and if not
downloads the corresponding zip file from the specified server, upacks it, and removes the .zip again.
\param file_path - FEMFX/car-body-tets
\param tmp_folder - a termporary folder that can be used to store intermediate products of the downloading process. Should be either non existent or empty.
\param folder - target folder where the files are stored in, e.g. assets
\param server - server from which the assets are download from as zip files. Must store the files in the same folder structure as they will be then stored locally.
\param acceptable_file_types - files types that are accepted. Other files types are ignored.
'''
def download_file_if_not_exists(file_path, tmp_folder, folder = "assets", server = "https://meshes.mailbase.info", \
                                acceptable_file_types = (".obj", ".mtl", ".off", ".node", ".ele", ".tet", ".bmp", ".png")):

    if not os.path.exists(folder):
        os.mkdir(folder)

    # search for file name
    
    # split path up
    # path_list = [FEMFX car-body-tets]
    path_list = file_path.split("/")
    
    # e.g.
    # path_to_file = FEMFX
    # file_name = car-body-tets
    path_to_file=""
    if len(path_list) > 1:
        path_to_file=path_list[0]
    
    file_name=path_list[len(path_list) - 1]

    for i in range(len(path_list)):
        if (i != 0 and i < len(path_list) - 1):
            path_to_file = path_to_file + "/" + path_list[i]

    if debug_output:
        print ("path to file = " + path_to_file + "\n" + \
               "file_name = " + file_name)

    # check if the file exists by trying out all supported extensions
    # check for:
    # FEMFX/car-body-tets.obj
    # FEMFX/car-body-tets.mtl
    # FEMFX/car-body-tets.off
    # etc.
    
    existing_file = ""
    for i in range(len(acceptable_file_types)):
        checked_file = folder + "/" + file_path + acceptable_file_types[i]
        if os.path.isfile(checked_file):
            existing_file = checked_file
            break
    
    if debug_output:
        print("existing_file = " + existing_file)

    if existing_file != "":
        if debug_output:
            print("File " + existing_file + "is already there. Done.")
        return
    
    # file_to_download = https://meshes.mailbase.info/FEMFX/car-body-tets.zip
    file_to_download = file_path + ".zip"
    print ("File " + file_path + " does not exist. Downloading file " + file_to_download + " from $server...")

    # zip_file = car-body-tets.zip
    zip_file=file_name + ".zip"

    # https://mailbase.info/meshes/car-body-tets.zip
    download_link = server + "/" + file_to_download

    # create _tmp folder
    if not os.path.exists(tmp_folder):
        os.mkdir(tmp_folder)
    
    with system_helper.cd(tmp_folder):
        if debug_output:
            print ("Download link = " + download_link)
            
        # download the file and put it in the current folder            
        try:
            r = requests.get(download_link)
            with open(zip_file, 'wb') as outfile:
                outfile.write(r.content)
        except Exception as e:
            print (e)
            print ("Download unavailable at " + download_link + ". Aborting...")
            return
        
        print("Download complete, unzip...")
        
        with zipfile.ZipFile(zip_file, 'r') as zip_ref:
            zip_ref.extractall()
            
        os.remove(zip_file)
        
        if os.path.exists("LICENSE.txt"):
            os.remove("LICENSE.txt")
        
        path = "../" + folder + "/" + path_to_file
        if not os.path.exists(path):
            Path(path).mkdir(parents=True, exist_ok=True)
            
        system_helper.copytree("./", "../" + folder + "/" + path_to_file + "/")
        
        print ("Installation of " + file_path + " is done.")

'''
Downloads the given files from a webserver, extracts them and moves them in the given folder.
The corresponding files must be available on the server relative to the given path, e.g. FEMFX/car-body-tets
must be availbale under https://meshes.mailbase.info/FEMFX/car-body-tets
Then, stores the files locally in <folder>/FEMFX/
\param files - list of files that are downloaded.
\param folder - target folder where the include libraries are stored in, e.g. assets
\param server - server from which the assets are download from as zip files. Must store the files in the same folder structure as they will be then stored locally.
\param acceptable_file_types - files types that are accepted. Other files types are ignored.
'''
def download_files_if_not_exist(files, folder = "assets", server = "https://meshes.mailbase.info", \
                                acceptable_file_types = (".obj" ".mtl" ".off" ".node" ".ele" ".tet" ".bmp" ".png")):
    
    # Create there is already a _tmp folder, remove all files in it.
    tmp_folder = "_tmp"
    while os.path.exists(tmp_folder):
        tmp_folder = "_" + tmp_folder
    
    if debug_output:
        print ("tmp_folder = " + tmp_folder)
        
    for f in files:
        download_file_if_not_exists(f, tmp_folder)
    
    if os.path.exists(tmp_folder):
        shutil.rmtree(tmp_folder)



