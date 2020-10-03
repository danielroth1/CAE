
import os
import shutil
import system_helper
import zipfile
import requests


'''
Downloads a header only library as zip file and extracts the relevant include folder in the extern/ folder.
\param 1 - target folder name: /extern/<name>, e.g. "cgal"
\param 2 - include folder in downloaded library, e.g. "CGAL-5.0.2/include"
\param 3 - zip file name, e.g. "CGAL-5.0.2-library.zip"
\param 4 - link to .zip file, e.g. "https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2-library.zip"
'''

extern_folder="extern"

debug_output = False

class BinaryLibDescription:
    '''
    \param target_folder, e.g. "freeglut"
    \param zip_file, e.g. "freeglut.zip"
    \param download_link, e.g. "https://meshes.mailbase.info/libs/freeglut.zip"
    '''
    def __init__(self, target_folder, zip_file, download_link):
        self.target_folder = target_folder
        self.zip_file = zip_file
        self.download_link = download_link

def download_binary_lib(target_folder, zip_file, download_link, tmp_folder, extern_folder = "extern", verbose = True):    
    '''
    \param target_folder, e.g. "freeglut"
    \param zip_file, e.g. "freeglut.zip"
    \param download_link, e.g. "https://meshes.mailbase.info/libs/freeglut.zip"
    \param tmp_folder - a termporary folder that can be used to store intermediate products of the downloading process. Should be either non existent or empty.
    \param extern_folder - folder in which the libraries are stored in, e.g. "extern"
    \param verbose - additional information are printed in output
    '''

    path = extern_folder + "/" + target_folder
    if os.path.exists(path):
        if debug_output:
            print(target_folder + " already exists. Skipping...")
    else:
        if not os.path.exists(tmp_folder):
            os.mkdir(tmp_folder)
        with system_helper.cd(tmp_folder):
            if verbose:
                print (target_folder + " not found, downloading...")
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
                
            if debug_output:
                print ("Downloading complete.")
            
            with zipfile.ZipFile(zip_file, 'r') as zip_ref:
                zip_ref.extractall()
            os.remove(zip_file)
            
            os.makedirs("../extern/" + target_folder, exist_ok=True)
            system_helper.copytree(target_folder, "../extern/" + target_folder)
            if debug_output:
                print("Successfully downloaded and installed " + target_folder)

def download_binary_libs(binary_lib_descriptions, extern_folder = "extern", verbose = True):
    '''
    Downloads a binary library as zip file and extracts the folder to the extern_folder.
    \param binary_lib_descriptions - list of BinaryLibDescriptions that 
        are used to download the binary libraries.
    \param extern_folder - folder in which the libraries are stored in, e.g. "extern"
    \param verbose - additional information are printed in output
    '''

    # create a tmp folder that doesn't exist yet. Simply appends _
    tmp_folder = "_tmp"
    while os.path.exists(tmp_folder):
        tmp_folder = "_" + tmp_folder
        
    if not os.path.exists(extern_folder):
        os.mkdir(extern_folder)
        
    for d in binary_lib_descriptions:
        download_binary_lib(d.target_folder, d.zip_file, d.download_link, tmp_folder, extern_folder, verbose)

    if os.path.isdir(tmp_folder):
        shutil.rmtree(tmp_folder)
        