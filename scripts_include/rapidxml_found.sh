#!/bin/bash

installed_rapidxml_md5="d698fa3e3547874766de5ba8cb2177c9"

computed_rapidxml_eigen_md5=`find include/rapidxml -type f -name '*.hpp' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`

if [ "$computed_rapidxml_eigen_md5" == "$installed_rapidxml_md5" ]
then
  exit 0
else
  exit 1
fi
