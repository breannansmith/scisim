#!/bin/bash

# Verify that md5sum is installed
command -v md5sum >/dev/null 2>&1 || command -v md5 >/dev/null 2>&1 || { echo >&2 "Error, please install md5 or md5sum and rerun get_dependencies.sh."; exit 1; }
if command -v md5 > /dev/null 2>&1 ; then
  md5sum() 
  {
    md5 "$@" | sed -e 's#^MD5 [(]\(.*\)[)] = \(.*\)$#\2 \1#' 
  }
  export -f md5sum 
fi

installed_sobogus_dir_md5="5f4f4b059f5b5db54a548479e7aa94cf"
installed_hpp_md5="5f6590e7e6ddb278f6e7740c3e097976"
installed_cpp_md5="73fba2af92aa42fc13f6bb9a5004b8f0"
installed_impl_md5="f6ccfaa3d971f99ba80363674d83448b"

# Check the installed headers
computed_installed_sobogus_dir_md5=`find include/sobogus -type f -name '*.[hc]pp' -exec bash -c 'md5sum "$0" "$@"' {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`
if [ "$computed_installed_sobogus_dir_md5" == "$installed_sobogus_dir_md5" ]
then
  exit 0
else
  exit 1
fi

# Check the local hpp file
computed_installed_hpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.hpp | cut -c -32`
if [ "$computed_installed_hpp_md5" == "$installed_hpp_md5" ]
then
  exit 0
else
  exit 1
fi

# Check the local cpp file
computed_installed_cpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.cpp | cut -c -32`
if [ "$computed_installed_cpp_md5" == "$installed_cpp_md5" ]
then
  exit 0
else
  exit 1
fi

# Check the local impl file
computed_installed_impl_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp | cut -c -32`
if [ "$computed_installed_impl_md5" == "$installed_impl_md5" ]
then
  exit 0
else
  exit 1
fi
