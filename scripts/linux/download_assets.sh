#!/bin/bash

# Disclaimer: This script is called by qmake or CMake. It is not necessary to call it manually.

# Download all the assets as .zip files from meshes.mailbase.info, unzips them, moves them in assets/*, and finally removes the .zip files.
# Already downloaded assets are skipped.
# The target folder corresponds to the specified ones, e.g.
# FEMFX/car-body-tets
# goes in assets/FEMFX/*

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

bash $DIR/download_file_if_not_exists.sh \
    "FEMFX/car-body-tets" \
    "FEMFX/car-body-tets-convex" \
    \
    "FEMFX/car-hood-tets" \
    "FEMFX/car-wheel0-tets" \
    "FEMFX/car-wheel1-tets" \
    "FEMFX/car-wheel2-tets" \
    "FEMFX/car-wheel3-tets" \
    \
    "nasa/advanced_crew_escape_suit" \
    "nasa/advanced_crew_escape_suit_convex" \
    \
    "primitives/cube_12k" \
    "primitives/cube_big_50k" \
    "primitives/cylinder_triagulated" \
    "primitives/floor_big_50k" \
    "primitives/floor_big_120k" \
    "fractal_terrain" \
    \
    "animals/Armadillo40k" \
    "animals/Bunny35k" \
    "animals/Frog19k" \
    \
    "textures/stonetiles_002_diff" \

