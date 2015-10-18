#!/bin/bash

command -v wget >/dev/null 2>&1 || { echo >&2 "Error, please install wget and rerun get_dependencies.sh."; exit 1; }
command -v git >/dev/null 2>&1 || { echo >&2 "Error, please install git and rerun get_dependencies.sh."; exit 1; }
command -v md5sum >/dev/null 2>&1 || { echo >&2 "Error, please install md5sum and rerun get_dependencies.sh."; exit 1; }

./scripts_include/get_eigen.sh
echo ""
./scripts_include/get_rapidxml.sh
echo ""
./scripts_include/get_sobogus.sh
echo ""
echo "All dependencies successfully installed."
echo ""
