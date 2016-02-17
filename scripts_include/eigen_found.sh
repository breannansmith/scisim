#!/bin/bash

installed_eigen_md5="730a5ac451cea99fc13b916c01eb27b2"

computed_installed_eigen_md5=`find include/eigen -type f -name '*.h' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`

if [ "$computed_installed_eigen_md5" == "$installed_eigen_md5" ]
then
  exit 0
else
  exit 1
fi
