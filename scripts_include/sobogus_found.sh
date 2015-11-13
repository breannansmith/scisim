#!/bin/bash

installed_sobogus_dir_md5="6e13d2b085f6893e75151d2a611ce8fd"
installed_hpp_md5="1bcfc6b838e0b84652c66e02550df5d3"
installed_cpp_md5="ce26cfc11cb73e8c156f61cca487c11b"
installed_impl_md5="b94ef20cd897ba9c4954f8f77c0fcbd5"

# Check the installed headers
computed_installed_sobogus_dir_md5=`find include/sobogus -type f -name '*.[hc]pp' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`
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
computed_installed_hpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.cpp | cut -c -32`
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
