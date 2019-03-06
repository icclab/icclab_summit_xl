#!/bin/bash

pwd
git clone --depth=1 https://github.com/icclab/rosdocked-irlab --branch=master ~/rosdocked-irlab
cd ~/rosdocked-irlab/workspace_included
./build.sh
docker images
