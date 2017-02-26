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

installed_sobogus_dir_md5="e8b8f43311b47248e93dbab09ff80b4d"
installed_hpp_md5="4a4c172dac2fed99bde9ae2104d1d7b8"
installed_cpp_md5="b18f0cf99d78d6105f3c7f24470a9f4c"
installed_impl_md5="1143d047b297566c96c9678b6be41302"

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
