#!/bin/bash

# Verify that md5sum is installed
command -v md5sum >/dev/null 2>&1 || command -v md5 >/dev/null 2>&1 || { echo >&2 "Error, please install md5 and rerun get_dependencies.sh."; exit 1; }
if command -v md5 > /dev/null 2>&1 ; then
  md5sum() 
  {
    md5 "$@" | sed -e 's#^MD5 [(]\(.*\)[)] = \(.*\)$#\2 \1#' 
  }
  export -f md5sum 
fi

installed_eigen_md5="730a5ac451cea99fc13b916c01eb27b2"

computed_installed_eigen_md5=`find include/eigen -type f -name '*.h' -exec bash -c 'md5sum "$0" "$@"' {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`

if [ "$computed_installed_eigen_md5" == "$installed_eigen_md5" ]
then
  exit 0
else
  exit 1
fi
