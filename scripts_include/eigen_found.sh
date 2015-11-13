#!/bin/bash

installed_eigen_md5="6612d555d442c4286a59cf52c7b0fa2e"

computed_installed_eigen_md5=`find include/eigen -type f -name '*.h' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`

if [ "$computed_installed_eigen_md5" == "$installed_eigen_md5" ]
then
  exit 0
else
  exit 1
fi
